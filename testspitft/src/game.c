#include "game.h"
#include "display.h"
#include "motor.h"

// -----------------------------------------------------------------------------
// External functions from other files
// These must NOT be static in the file where they are defined.
// -----------------------------------------------------------------------------
extern uint32_t millis(void);
extern void delay_ms(uint32_t ms);

extern void Joystick_Read(uint16_t *x, uint16_t *y, uint8_t *pressed);

extern void Display_ShowIdleScreen(void);
extern void Display_ShowPlayerTurn(player_t player, uint32_t time_remaining_ms);
extern void Display_ShowCheckingBoard(void);
extern void Display_ShowWinner(player_t winner);
extern void Display_ShowDraw(void);
extern void Hardware_ScanBoard(uint8_t scanned_board[9]);

extern void Motor_Enable(void);
extern void Motor_Disable(void);
extern void Motor_MoveX(motor_dir_t dir, uint32_t steps);
extern void Motor_MoveY(motor_dir_t dir, uint32_t steps);
extern void Motor_MoveZ(motor_dir_t dir, uint32_t steps);
extern void Motor_MoveClaw(motor_dir_t dir, uint32_t steps);

#define X_LIMIT_GPIO GPIOA
#define X_LIMIT_PIN 11

#define XLIM_NONE 0
#define XLIM_POS 1
#define XLIM_NEG 2

// -----------------------------------------------------------------------------
// Static game context
// -----------------------------------------------------------------------------
static game_context_t game;

// -----------------------------------------------------------------------------
// Winning line lookup table
// -----------------------------------------------------------------------------
static const uint8_t win_lines[8][3] = {
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8},
    {0, 3, 6},
    {1, 4, 7},
    {2, 5, 8},
    {0, 4, 8},
    {2, 4, 6}};

// -----------------------------------------------------------------------------
// Board helpers
// -----------------------------------------------------------------------------
static void Board_Init(void)
{
    for (uint8_t i = 0; i < 9; i++)
    {
        game.board[i] = PLAYER_NONE;
    }

    game.move_count = 0;
    game.last_move_cell = 0xFF;
}

static void Board_RecountMoves(void)
{
    uint8_t count = 0;
    uint8_t last = 0xFF;

    for (uint8_t i = 0; i < 9; i++)
    {
        if (game.board[i] != PLAYER_NONE)
        {
            count++;
            last = i;
        }
    }

    game.move_count = count;
    game.last_move_cell = last;
}

static player_t Board_CheckWinner(void)
{
    for (uint8_t line = 0; line < 8; line++)
    {
        uint8_t a = win_lines[line][0];
        uint8_t b = win_lines[line][1];
        uint8_t c = win_lines[line][2];

        if (game.board[a] != PLAYER_NONE &&
            game.board[a] == game.board[b] &&
            game.board[b] == game.board[c])
        {
            return (player_t)game.board[a];
        }
    }

    return PLAYER_NONE;
}

static uint8_t Board_IsDraw(void)
{
    return (game.move_count >= 9 && Board_CheckWinner() == PLAYER_NONE);
}

// -----------------------------------------------------------------------------
// TODO hook: full board scan
// Replace this with real sensor scanning later.
// For now it is just a placeholder that leaves the board unchanged.
// -----------------------------------------------------------------------------
static void Board_ScanAllCells(uint8_t scanned_board[9])
{
    Hardware_ScanBoard(scanned_board);
}

static void Board_UpdateFromScan(void)
{
    uint8_t scanned_board[9];

    Board_ScanAllCells(scanned_board);

    for (uint8_t i = 0; i < 9; i++)
    {
        game.board[i] = scanned_board[i];
    }

    Board_RecountMoves();
}

void X_Limit_Checker(uint8_t dir, uint8_t *xlim_prev)
{
    uint8_t xlim_current;

    xlim_current = ((X_LIMIT_GPIO->IDR & (1 << X_LIMIT_PIN)) == 0);

    if (!xlim_current && *xlim_prev)
    {
        *xlim_prev = XLIM_NONE;
    }

    else if (xlim_current && dir == DIR_FORWARD && !*xlim_prev)
    {
        *xlim_prev = XLIM_POS;
    }

    else if (xlim_current && dir == DIR_BACKWARD && !*xlim_prev)
    {
        *xlim_prev = XLIM_NEG;
    }
}
// -----------------------------------------------------------------------------
// Update Claw Movemenet From Joystick Input
// -----------------------------------------------------------------------------
static uint8_t dir = 0;
static uint8_t xlim = 0;

static void Claw_UpdateFromJoystick(uint16_t x, uint16_t y)
{

    X_Limit_Checker(dir, &xlim);

    if (x > y && x > 3000 && xlim != XLIM_POS)
    {
        Motor_Step(AXIS_X, DIR_FORWARD, 10);
        dir = DIR_FORWARD;
    }
    else if (y > x && y > 3000)
    {
        Motor_Step(AXIS_Y, DIR_FORWARD, 10);
        dir = DIR_FORWARD;
    }
    else if (x < y && x < 1500 && xlim != XLIM_NEG)
    {
        Motor_Step(AXIS_X, DIR_BACKWARD, 10);
        dir = DIR_BACKWARD;
    }
    else if (y < x && y < 1500)
    {
        Motor_Step(AXIS_Y, DIR_BACKWARD, 10);
        dir = DIR_BACKWARD;
    }
}

// -----------------------------------------------------------------------------
// State transition helper
// -----------------------------------------------------------------------------
static void Game_ChangeState(game_state_t new_state)
{
    game.current_state = new_state;
    game.turn_start_time = millis();

    switch (new_state)
    {
    case STATE_IDLE:
        Display_ShowIdleScreen();
        break;

    case STATE_PLAYER1_TURN_GRAB:
        game.active_player = PLAYER_1;
        Display_ShowPlayerTurn(PLAYER_1, PLAYER_TURN_TIME_MS);
        break;

    case STATE_PLAYER2_TURN_GRAB:
        game.active_player = PLAYER_2;
        Display_ShowPlayerTurn(PLAYER_2, PLAYER_TURN_TIME_MS);
        break;

    case STATE_CHECK_BOARD:
        Display_ShowCheckingBoard();
        break;

    case STATE_PLAYER1_WIN:
        Display_ShowWinner(PLAYER_1);
        break;

    case STATE_PLAYER2_WIN:
        Display_ShowWinner(PLAYER_2);
        break;

    case STATE_DRAW:
        Display_ShowDraw();
        break;

    case STATE_RESET:
    default:
        break;
    }
}

// -----------------------------------------------------------------------------
// State handlers
// -----------------------------------------------------------------------------
static void Handle_IdleState(void)
{
    uint16_t jx, jy;
    uint8_t pressed;
    Display_ShowIdleScreen();
    Joystick_Read(&jx, &jy, &pressed);

    if (pressed)
    {
        Board_Init();
        game.active_player = PLAYER_1;
        Game_ChangeState(STATE_PLAYER1_TURN_GRAB);
        delay_ms(200);
    }
}

static void Handle_PlayerTurnState_Grab(void)
{
    uint32_t elapsed = millis() - game.turn_start_time;
    uint32_t remaining =
        (elapsed >= PLAYER_TURN_TIME_MS) ? 0 : (PLAYER_TURN_TIME_MS - elapsed);

    uint16_t jx, jy;
    uint8_t pressed;

    Joystick_Read(&jx, &jy, &pressed);
    Claw_UpdateFromJoystick(jx, jy);

    Display_ShowPlayerTurn(game.active_player, remaining);

    if (pressed)
    {
        Claw_Grab_Token();
        
        if (game.active_player == PLAYER_1) {
            Game_ChangeState(STATE_PLAYER1_TURN_DROP);
        }
        else if (game.active_player == PLAYER_2) {
            Game_ChangeState(STATE_PLAYER2_TURN_DROP);
        }

        delay_ms(200);
        return;
    }

    if (elapsed >= PLAYER_TURN_TIME_MS)
    {
        Claw_Grab_Token();

        if (game.active_player == PLAYER_1) {
            Game_ChangeState(STATE_PLAYER1_TURN_DROP);
        }
        else if (game.active_player == PLAYER_2) {
            Game_ChangeState(STATE_PLAYER2_TURN_DROP);
        }
    }
}

static void Handle_PlayerTurnState_Drop(void)
{
    uint32_t elapsed = millis() - game.turn_start_time;
    uint32_t remaining =
        (elapsed >= PLAYER_TURN_TIME_MS) ? 0 : (PLAYER_TURN_TIME_MS - elapsed);

    uint16_t jx, jy;
    uint8_t pressed;

    Joystick_Read(&jx, &jy, &pressed);
    Claw_UpdateFromJoystick(jx, jy);

    Display_ShowPlayerTurn(game.active_player, remaining);

    if (pressed)
    {
        Claw_Drop_Token();

        Game_ChangeState(STATE_CHECK_BOARD);
        delay_ms(200);
        return;
    }

    if (elapsed >= PLAYER_TURN_TIME_MS)
    {
        Claw_Drop_Token();

        Game_ChangeState(STATE_CHECK_BOARD);
    }
}

static void Handle_CheckBoardState(void)
{
    Board_UpdateFromScan();

    player_t winner = Board_CheckWinner();

    if (winner == PLAYER_1)
    {
        Game_ChangeState(STATE_PLAYER1_WIN);
        return;
    }

    if (winner == PLAYER_2)
    {
        Game_ChangeState(STATE_PLAYER2_WIN);
        return;
    }

    if (Board_IsDraw())
    {
        Game_ChangeState(STATE_DRAW);
        return;
    }

    // Always switch turns after board check
    if (game.active_player == PLAYER_1)
    {
        Game_ChangeState(STATE_PLAYER2_TURN_GRAB);
    }
    else
    {
        Game_ChangeState(STATE_PLAYER1_TURN_GRAB);
    }
}

static void Handle_EndState(void)
{
    if ((millis() - game.turn_start_time) >= TRANSITION_TIME_MS)
    {
        Game_ChangeState(STATE_RESET);
    }
}

static void Handle_ResetState(void)
{
    Board_Init();
    game.active_player = PLAYER_1;
    Game_ChangeState(STATE_IDLE);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void Game_Init(void)
{
    Board_Init();
    Game_ChangeState(STATE_IDLE);
    game.active_player = PLAYER_1;
    game.current_state = STATE_IDLE;
    game.turn_start_time = 0;
}

void Game_Update(void)
{
    switch (game.current_state)
    {
    case STATE_IDLE:
        Handle_IdleState();
        break;

    case STATE_PLAYER1_TURN_GRAB:
    case STATE_PLAYER2_TURN_GRAB:
        Handle_PlayerTurnState_Grab();
        break;

    case STATE_PLAYER1_TURN_DROP:
    case STATE_PLAYER2_TURN_DROP:
        Handle_PlayerTurnState_Drop();
        break;

    case STATE_CHECK_BOARD:
        Handle_CheckBoardState();
        break;

    case STATE_PLAYER1_WIN:
    case STATE_PLAYER2_WIN:
    case STATE_DRAW:
        Handle_EndState();
        break;

    case STATE_RESET:
        Handle_ResetState();
        break;

    default:
        Game_ChangeState(STATE_IDLE);
        break;
    }
}

game_state_t Game_GetState(void)
{
    return game.current_state;
}

player_t Game_GetActivePlayer(void)
{
    return game.active_player;
}

const game_context_t *Game_GetContext(void)
{
    return &game;
}
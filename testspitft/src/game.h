#ifndef GAME_H
#define GAME_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// Timing constants
// -----------------------------------------------------------------------------
#define PLAYER_TURN_TIME_MS 5000
#define TRANSITION_TIME_MS  5000

// -----------------------------------------------------------------------------
// Game State Definitions
// -----------------------------------------------------------------------------
typedef enum {
    STATE_IDLE = 0,
    STATE_PLAYER1_TURN_GRAB,
    STATE_PLAYER2_TURN_GRAB,
    STATE_PLAYER1_TURN_DROP,
    STATE_PLAYER2_TURN_DROP,
    STATE_CHECK_BOARD,
    STATE_PLAYER1_WIN,
    STATE_PLAYER2_WIN,
    STATE_DRAW,
    STATE_RESET
} game_state_t;

typedef enum {
    PLAYER_NONE = 0,
    PLAYER_1 = 1,
    PLAYER_2 = 2
} player_t;

// -----------------------------------------------------------------------------
// Game Data Structure
// -----------------------------------------------------------------------------
typedef struct {
    game_state_t current_state;
    player_t active_player;
    uint8_t board[9];
    uint32_t turn_start_time;
    uint8_t move_count;
    uint8_t last_move_cell;
} game_context_t;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void Game_Init(void);
void Game_Update(void);
static void Game_ChangeState(game_state_t new_state);

game_state_t Game_GetState(void);
player_t Game_GetActivePlayer(void);
const game_context_t* Game_GetContext(void);

#endif
# JetsonHatdwareManagementBoard
    ┌───────────────────────────────────────────────────────────────────────────┐
    │               Debounce + Gesture (PRESS / LONG / DOUBLE)                  │
    └───────────────────────────────────────────────────────────────────────────┘

    Legend:
    raw_pressed   = immediate GPIO sample (after active_low) 0/1
    integ         = per-pin integrator counter [0..db_max]
    stable        = debounced state (0=released, 1=pressed)
    waiting_second= first short press released, waiting for 2nd press
    first_release = timestamp when first short press released
    pressed_at    = timestamp when stable became pressed
    long_fired    = LONG event already posted for this hold

    Sampling loop (every Ts ms, e.g. 5 ms)
    ─────────────────────────────────────────────────────────────────────────────

             ┌─────────────────────────────────────────────────────────┐
             │ 1) Read input                                           │
             │ raw_pressed = (GPIO read) converted to pressed=1/0      │
             └───────────────┬─────────────────────────────────────────┘
                             │
                             ▼
    ┌──────────────────────────────────────────────────────────────────────────┐
    │ 2) Debounce integrator (per pin, no shared sums!)                        │
    │                                                                          │
    │   if raw_pressed == 1: integ = min(integ+1, db_max)                      │
    │   if raw_pressed == 0: integ = max(integ-1, 0)                           │
    │                                                                          │
    │   if integ == db_max and stable == 0  → stable=1  (PRESS EDGE)           │
    │   if integ == 0      and stable == 1  → stable=0  (RELEASE EDGE)         │
    └───────────────┬──────────────────────────────────────────────────────────┘
                    │
                    ▼
    ┌───────────────────────────────┐
    │ 3) On PRESS EDGE (stable:0→1) │
    └───────────────────────────────┘
                │
                │ set pressed_at = now_ms
                │ long_fired = 0
                ▼
    ┌───────────────────────────────────────────────────────────────────────────┐
    │ 4) While HELD (stable==1): LONG press detection                           │
    │                                                                           │
    │   if stable==1 AND long_fired==0 AND (now_ms - pressed_at) >= long_ms     │
    │        → post LONG_PRESS event                                            │
    │        → long_fired = 1                                                   │
    │        → waiting_second = 0   (LONG cancels double-click tracking)        │
    └───────────────┬───────────────────────────────────────────────────────────┘
                    │
                    ▼
    ┌──────────────────────────────────┐
    │ 5) On RELEASE EDGE (stable:1→0)  │
    └──────────────────────────────────┘
                    │
            ┌───────┴────────────────────────────────────────────────────────┐
            │ if long_fired==1                                               │
            │    → do nothing (LONG already reported; release gives no event)│
            └───────┬────────────────────────────────────────────────────────┘
                    │ else (short press candidate)
                    ▼
    ┌───────────────────────────────────────────────────────────────────────────┐
    │ 6) Double press state machine (only for short presses)                    │
    │                                                                           │
    │   CASE A: waiting_second == 0                                             │
    │      → this is first short release                                        │
    │      → waiting_second = 1                                                 │
    │      → first_release = now_ms                                             │
    │                                                                           │
    │   CASE B: waiting_second == 1                                             │
    │      → this is a (potential) second short release                         │
    │      if (now_ms - first_release) <= double_window_ms                      │
    │          → post DOUBLE_PRESS event                                        │
    │          → waiting_second = 0                                             │
    │      else                                                                 │
    │          → too late: treat as new first release                           │
    │          → first_release = now_ms                                         │
    │          → waiting_second stays 1                                         │
    └───────────────┬───────────────────────────────────────────────────────────┘
                    │
                    ▼
    ┌───────────────────────────────────────────────────────────────────────────┐
    │ 7) Timeout finalization for SINGLE short press                            │
    │   (runs each tick while stable==0)                                        │
    │                                                                           │
    │   if waiting_second==1 AND stable==0 AND                                  │
    │      (now_ms - first_release) > double_window_ms                          │
    │        → post PRESSED event  (confirmed single click)                     │
    │        → waiting_second = 0                                               │
    └───────────────────────────────────────────────────────────────────────────┘


    State timeline examples
    ─────────────────────────────────────────────────────────────────────────────
    
    A) Single short press
    raw:     ____████____
    integ:   0 1 2 3 4(=db_max) ... 3 2 1 0
    stable:  ____████____           (edges at integ saturation)
    │   │
    │   └─ release edge → start waiting_second timer
    └─ press edge → pressed_at set
    window expires → PRESSED event posted
    
    B) Double press
    stable:  __██__██__
    │   │
    │   └─ 2nd release within window → DOUBLE_PRESS posted
    └─ 1st release starts waiting_second
    
    C) Long press
    stable:  __████████████____
    │
    └─ (now-pressed_at) >= long_ms → LONG_PRESS posted (once)
    release edge after long → no extra event

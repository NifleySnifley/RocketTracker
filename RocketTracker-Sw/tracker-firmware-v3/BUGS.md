## Known bugs:
- It is possible to get into a lockup state if the radio starts receiving, and `radio_tx` is called *while* the radio is receiving
  - Solutions: 
    - Find some way to have a `rx_started` function
    - Use a mutex to manage the radio, would need `rx_started` to lock for reception
    - **Recover from the lockup condition (timeout)**
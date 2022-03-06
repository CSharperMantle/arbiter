# arbiter-message

Protocol Buffers definition files for arbiter message interchange.

## Messages

* `SerialHostMessage`
* `SerialMonitorMessage`
* `WirelessClientMessage`
* `WirelessHostMessage`

## Message flow

### C/H handshake

1. A Client `A` is engaged. It probes for a Host `H` with `HANDSHAKE_REQUEST`.
2. `H` responds to `A` with `HANDSHAKE_ASSIGNMENT`, sending `A` its ID.
3. `A` acknowledges the assigned ID with `HANDSHAKE_ASSIGNMENT_CONFIRMATION`.

### M/H data sync

1. A Monitor `M` sends to a Host `H` a `QUERY_REQUEST`.
2. `H` sends a `PING` to each registered Client.
3. Clients respond with `PONG`.
4. `H` sends a `QUERY_CONFIRMATION` containing `clients` back to `M`.

### Arbitration

1. A Client `C` sends `ARBITRATION_REQUEST` to a Host `H`.
2. `H` judges if the request is valid and send `ARBITRATION_CONFIRMATION` if so.
3. `C` take actions to indicate a successful arbitration.
4. `H` sends Monitor `M` a `ARBITRATION_DONE` with `arbitration` to indicate a successful arbitration.

### Reset

1. A Monitor `M` sends `RESET_REQUEST`.
2. A Host `H` sends `RESET_REQUEST` to each Client and wait for their `RESET_CONFIRMATION`.
3. `H` returns a `RESET_CONFIRMATION` to `H`.

# Robot Controller State Machine

## Stawtes
- IDLE
- DRIVE
- LIFT_OP
- DUMP_OP
- FAULT
- E_STOP

## Transitions

### IDLE -> DRIVE
Conditions (AND):
- drive_enable == 1
- estop == 0
- battery_ok == true
- comms_ok == true
- no_active_fault == true

### IDLE -> LIFT_OP
Conditions:
- lift_request == 1
- estop == 0
- no_active_fault == true

### IDLE -> DUMP_OP
Conditions:
- dump_request == 1
- estop == 0
- no_active_fault == true

### DRIVE -> IDLE
Conditions:
- drive_enable == 0
- velocity == 0

### DRIVE -> LIFT_OP
Conditions:
- lift_request == 1
- velocity == 0  ← 주행 중 리프트 금지

### DRIVE -> DUMP_OP
Conditions:
- dump_request == 1
- velocity == 0

### LIFT_OP -> IDLE
Conditions:
- lift_complete == true
- lift_request == 0

### LIFT_OP -> FAULT
Conditions:
- lift_timeout (동작 시간 초과)
- lift_sensor_error

### DUMP_OP -> IDLE
Conditions:
- dump_complete == true
- dump_request == 0

### DUMP_OP -> FAULT
Conditions:
- dump_timeout
- dump_sensor_error

### ANY -> FAULT
Conditions:
- CAN timeout
- critical DTC 발생

### ANY -> E_STOP
Conditions:
- estop_button == 1  ← 최우선, 즉시 전이

### E_STOP -> IDLE
Conditions:
- estop_button == 0
- operator_ack == true  ← 수동 해제 필요

### FAULT -> IDLE
Conditions:
- no_active_fault == true
- operator_ack == true

## Safety Policy
- FAULT 또는 E_STOP 상태에서는 모든 출력(모터/밸브)을 0으로 강제한다.
- FAULT 해제는 오퍼레이터 확인(ACK) 이후에만 가능하다.


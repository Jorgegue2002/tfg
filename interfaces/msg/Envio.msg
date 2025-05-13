# Un mensaje que indica el estado de el envio y su destino (0–9)

string status    # state: “on”, “off”, “waiting_for_order”, “waiting_for_discharge”

#Se puede hacer asi tambien
#uint8 state
#uint8 STATE_OFF=0
#uint8 STATE_ON=1
#uint8 STATE_WAITING_FOR_ORDER=2
#uint8 STATE_WAITING_FOR_DISCHARGE=3


uint8  destination  # Número de destino del 0 a 9 (El 0 esta reservado para el lugar del dock)

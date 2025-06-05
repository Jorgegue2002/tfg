# interfaces/msg/Envio.msg
# Un mensaje que indica el tipo de envio y su destino (0–9)

# status puede tomar uno de estos valores:
# - "deliver"             → ir directamente a un punto para entregar
# - "pickup_and_deliver"   → primero pasar por el dock (0) a recoger, luego ir a destination
# - "go_to_dock"           → ir al dock (destino 0) independientemente del destino que venga en el mensaje
# - "waiting_for_order"    → el robot está en el dock esperando que carguen algo o a recibir una orden
# - "waiting_for_discharge"→ el robot llegó a destino distinto de 0 y espera que descarguen

string status
uint8  destination   # 0 = dock, 1–9 = otros puntos

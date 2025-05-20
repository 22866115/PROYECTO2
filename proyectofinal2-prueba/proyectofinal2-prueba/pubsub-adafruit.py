import serial
import time
from Adafruit_IO import MQTTClient

# Configuración Adafruit IO
ADAFRUIT_IO_USERNAME = "Lara115mx"
ADAFRUIT_IO_KEY = "aio_tCbI14XyFmy9jZQHtIqEOFKaMBoc"

# Configuración puerto serial (ajustar puerto COM según tu sistema)
SERIAL_PORT = 'COM9'  # Windows
# SERIAL_PORT = '/dev/ttyUSB0'  # Linux
BAUD_RATE = 9600

# Feeds para cada dedo
FEEDS = {
    'DEDO1': 'dedo1',
    'DEDO2': 'dedo2', 
    'DEDO3': 'dedo3',
    'DEDO4': 'dedo4',
    'DEDO5': 'dedo5'
}

# Inicializar conexión serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Esperar a que se establezca la conexión
except serial.SerialException as e:
    print(f"Error al abrir puerto serial: {e}")
    exit(1)

# Callbacks Adafruit IO
def connected(client):
    print("Conectado a Adafruit IO!")
    for feed in FEEDS.values():
        client.subscribe(feed)

def disconnected(client):
    print("Desconectado de Adafruit IO!")
    sys.exit(1)

def message(client, feed_id, payload):
    print(f"Recibido {payload} de {feed_id}")
    
    # Determinar qué dedo mover
    dedo = None
    for key, value in FEEDS.items():
        if feed_id == value:
            dedo = key
            break
    
    if dedo:
        try:
            posicion = int(payload)
            if 0 <= posicion <= 255:
                # Enviar comando al Arduino: "DEDOXPPP\n" (X=1-5, PPP=000-255)
                comando = f"{dedo[-1]}{posicion:03d}\n"
                ser.write(comando.encode())
                print(f"Enviado a Arduino: {comando.strip()}")
            else:
                print("Error: Posición debe estar entre 0-255")
        except ValueError:
            print("Error: Payload debe ser un número")

# Configurar cliente MQTT
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Conectar e iniciar loop
client.connect()
client.loop_background()

print("Sistema listo. Esperando comandos...")

try:
    while True:
        # Leer feedback del Arduino si es necesario
        if ser.in_waiting:
            linea = ser.readline().decode().strip()
            print(f"Recibido de Arduino: {linea}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Cerrando programa...")
    ser.close()
    client.disconnect()
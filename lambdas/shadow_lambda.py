import json
import boto3

iot_data = boto3.client('iot-data', region_name='us-east-2')

def calculate_states(temperature_state):
    match temperature_state:
        case 0:
            light_state = 1
            cooler_state = 0
        case 1:
            light_state = 0
            cooler_state = 0
        case 2:
            light_state = 0
            cooler_state = 1
        case _:
            raise ValueError("Valor de temperatureState no válido")
    
    return light_state, cooler_state

def lambda_handler(event, context):
    try:
        # Validar claves en el evento
        try:
            thing_name = event['thing_name']
            temperature_state = event['temperatureState']
        except KeyError as e:
            return {
                "statusCode": 400,
                "body": f"Clave faltante en el evento: {str(e)}"
            }
        
        # Calcular los estados
        light_state, cooler_state = calculate_states(temperature_state)
        
        # Construir el payload
        payload = {
            "state": {
                "desired": {
                    "lightState": light_state,
                    "coolerState": cooler_state
                }
            }
        }
        
        # Publicar en el shadow
        response = iot_data.update_thing_shadow(
            thingName=thing_name,
            payload=json.dumps(payload)
        )
        
        # Leer y devolver respuesta
        shadow_response = json.loads(response['payload'].read())
        return {
            "statusCode": 200,
            "body": f"Shadow actualizado correctamente para {thing_name}",
            "response": shadow_response
        }
    
    except Exception as e:
        import traceback
        return {
            "statusCode": 500,
            "body": f"Error al actualizar el shadow: {str(e)}",
            "details": traceback.format_exc()
        }

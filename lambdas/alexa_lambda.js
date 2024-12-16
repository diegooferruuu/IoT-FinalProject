const Alexa = require('ask-sdk-core');
const AWS = require('aws-sdk');
const IotData = new AWS.IotData({ endpoint: 'a1o8cg6i3hlsiy-ats.iot.us-east-2.amazonaws.com' });
const dynamoDb = new AWS.DynamoDB.DocumentClient();

// Función para obtener el nombre del "thing" desde DynamoDB según el userId de Alexa
async function getThingName(userId) {
    const params = {
        TableName: 'UserThings',  // El nombre de tu tabla de DynamoDB
        Key: { userId: userId }
    };

    try {
        const result = await dynamoDb.get(params).promise();
        return result.Item ? result.Item.thing_name : null;
    } catch (error) {
        console.error('Error al obtener el thingName de DynamoDB:', error);
        throw new Error('No se pudo obtener el thingName');
    }
}

// Shadow Parameters for updating states
async function getShadowParams(userId) {
    const thingName = await getThingName(userId);
    console.log(thingName);
    if (!thingName) {
        throw new Error('Thing name no encontrado');
    }
    return { thingName: thingName };
}

// Function to get the current state of the Shadow
function getShadowPromise(params) {
    return new Promise((resolve, reject) => {
        IotData.getThingShadow(params, (err, data) => {
            if (err) {
                console.log(err, err.stack);
                reject('Error al obtener el shadow: ' + err.message);
            } else {
                resolve(JSON.parse(data.payload));
            }
        });
    });
}

// Function to update the desired state of the Shadow
function updateShadowPromise(params) {
    return new Promise((resolve, reject) => {
        IotData.updateThingShadow(params, (err, data) => {
            if (err) {
                console.log(err, err.stack);
                reject('Error al actualizar el shadow: ' + err.message);
            } else {
                resolve(data);
            }
        });
    });
}

// LaunchRequest Handler
const LaunchRequestHandler = {
  canHandle(handlerInput) {
      return Alexa.getRequestType(handlerInput.requestEnvelope) === 'LaunchRequest';
  },
  handle(handlerInput) {
      const speakOutput = 'Bienvenido al sistema de ventilación inteligente. Puedes preguntar por el estado de temperatura o cambiar la velocidad del ventilador. ¿Qué deseas hacer?';
      return handlerInput.responseBuilder.speak(speakOutput).reprompt(speakOutput).getResponse();
  }
};

// Handler para consultar el estado de la temperatura
const CheckTemperatureStateHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'CheckTemperatureStateIntent';
    },
    async handle(handlerInput) {
        let speakOutput = 'No se pudo obtener el estado de la temperatura.';
        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);
            const result = await getShadowPromise(ShadowParams);
            const tempState = result.state.reported.temperatureState;

            switch (tempState) {
                case 0:
                    speakOutput = 'La temperatura está fría, debajo de los 35 grados.';
                    break;
                case 1:
                    speakOutput = 'La temperatura está en estado ideal, entre 35 y 40.9 grados.';
                    break;
                case 2:
                    speakOutput = 'La temperatura está caliente, de los 41 grados para arriba.';
                    break;
                default:
                    speakOutput = 'No se pudo determinar el estado de la temperatura.';
            }
        } catch (error) {
            console.log(error);
        }

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const CheckIncubatorModeStateHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'CheckIncubatorModeStateIntent';
    },
    async handle(handlerInput) {
        let speakOutput = 'No se pudo obtener el estado de la incubadora.';
        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);
            const result = await getShadowPromise(ShadowParams);
            const incubatorM = result.state.reported.incubatorMode;

            switch (incubatorM) {
                case 0:
                    speakOutput = 'La incubadora está manual.';
                    break;
                case 1:
                    speakOutput = 'La incubadora esta automatica.';
                    break;
                default:
                    speakOutput = 'No se pudo determinar el estado de la incubadora.';
            }
        } catch (error) {
            console.log(error);
        }

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const CheckCoolerStateHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'CheckCoolerStateIntent';
    },
    async handle(handlerInput) {
        let speakOutput = 'No se pudo obtener el estado del ventilador.';
        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);
            const result = await getShadowPromise(ShadowParams);
            const coolerS = result.state.desired.coolerState;

            switch (coolerS) {
                case 0:
                    speakOutput = 'El ventilador esta apagado';
                    break;
                case 1:
                    speakOutput = 'El ventilador esta prendido.';
                    break;
                default:
                    speakOutput = 'No se pudo determinar el estado de la incubadora.';
            }
        } catch (error) {
            console.log(error);
        }

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const CheckLightStateHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'CheckLightStateIntent';
    },
    async handle(handlerInput) {
        let speakOutput = 'No se pudo obtener el estado del foco.';
        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);
            const result = await getShadowPromise(ShadowParams);
            const lightS = result.state.desired.lightState;

            switch (lightS) {
                case 0:
                    speakOutput = 'El foco incandescente esta apagado';
                    break;
                case 1:
                    speakOutput = 'El foco incandescente esta prendido.';
                    break;
                default:
                    speakOutput = 'No se pudo determinar el estado del foco.';
            }
        } catch (error) {
            console.log(error);
        }

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const ChangeLightStateManualHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'ChangeLightStateManualIntent';
    },
    async handle(handlerInput) {
        let light;
        const lightS = handlerInput.requestEnvelope.request.intent.slots.lightState.value;

        if (lightS === "enciende") { 
            light = 1; 
        } else { 
            light = 0; 
        }

        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);

            const payload = {
                state: {
                    desired: {
                        lightState: light,
                        incubatorMode: 0  // Modo manual
                    }
                }
            };

            const params = {
                ...ShadowParams,
                payload: JSON.stringify(payload)
            };

            await updateShadowPromise(params);
            const speakOutput = `${lightS} el foco funcionó`;
            return handlerInput.responseBuilder
                .speak(speakOutput)
                .reprompt(speakOutput)
                .getResponse();
        } catch (error) {
            console.error(error);
            return handlerInput.responseBuilder
                .speak('Hubo un error al cambiar el estado del foco. Por favor, inténtalo de nuevo.')
                .getResponse();
        }
    }
};


const ChangeCoolerStateManualHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'ChangeCoolerStateManualIntent';
    },
    async handle(handlerInput) {
        let cooler;
        const coolerS = handlerInput.requestEnvelope.request.intent.slots.coolerState.value;

        if (coolerS === "enciende") { 
            cooler = 1; 
        } else { 
            cooler = 0; 
        }

        try {
            const userId = handlerInput.requestEnvelope.context.System.user.userId;
            const ShadowParams = await getShadowParams(userId);

            const payload = {
                state: {
                    desired: {
                        coolerState: cooler,
                        incubatorMode: 0  // Modo manual
                    }
                }
            };

            const params = {
                ...ShadowParams,
                payload: JSON.stringify(payload)
            };

            await updateShadowPromise(params);
            const speakOutput = `${coolerS} el ventilador funcionó`;
            return handlerInput.responseBuilder
                .speak(speakOutput)
                .reprompt(speakOutput)
                .getResponse();
        } catch (error) {
            console.error(error);
            return handlerInput.responseBuilder
                .speak('Hubo un error al cambiar el estado del ventilador. Por favor, inténtalo de nuevo.')
                .getResponse();
        }
    }
};


// Handler para cambiar el modo a automático
const ChangeAutoModeHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'ChangeAutoModeIntent';
    },
    async handle(handlerInput) {
        const userId = handlerInput.requestEnvelope.context.System.user.userId;
        const ShadowParams = await getShadowParams(userId);

        const payload = {
            state: {
                desired: {
                    incubatorMode: 1  // Configura el modo automático a 1 (automático)
                }
            }
        };

        const params = {
            ...ShadowParams,
            payload: JSON.stringify(payload)
        };

        try {
            await updateShadowPromise(params);
            const speakOutput = 'El modo automático se ha activado.';
            return handlerInput.responseBuilder
                .speak(speakOutput)
                .reprompt(speakOutput)
                .getResponse();
        } catch (error) {
            console.log(error);
            return handlerInput.responseBuilder
                .speak('Hubo un error al activar el modo automático.')
                .getResponse();
        }
    }
};

// Otros handlers estándar
const HelpIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.HelpIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'Puedes pedirme consultar o cambiar el estado de la temperatura y el servomotor. ¿Qué deseas hacer?';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

const CancelAndStopIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && (Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.CancelIntent'
                || Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.StopIntent');
    },
    handle(handlerInput) {
        const speakOutput = '¡Hasta luego!';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .getResponse();
    }
};

const ErrorHandler = {
    canHandle() {
        return true;
    },
    handle(handlerInput, error) {
        console.log(`Error: ${error.message}`);
        return handlerInput.responseBuilder
            .speak('Hubo un problema. Intenta de nuevo.')
            .getResponse();
    }
};

// Exporta la skill con todos los handlers
exports.handler = Alexa.SkillBuilders.custom()
    .addRequestHandlers(
        LaunchRequestHandler,
        CheckTemperatureStateHandler,
        ChangeAutoModeHandler,
        HelpIntentHandler,
        CancelAndStopIntentHandler,
        CheckIncubatorModeStateHandler,
        CheckCoolerStateHandler,
        CheckLightStateHandler,
        ChangeCoolerStateManualHandler,
        ChangeLightStateManualHandler
    )
    .addErrorHandlers(ErrorHandler)
    .lambda();

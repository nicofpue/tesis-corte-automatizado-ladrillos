#include <Arduino.h>

#include "config.h"
#include "encoder.h"


// ============================================================
// ENCODER
// ============================================================

static volatile long encoderCount = 0;


// Protección para acceso desde ISR y programa principal
static portMUX_TYPE encoderMux =
    portMUX_INITIALIZER_UNLOCKED;


// ============================================================
// INTERRUPCION ENCODER
// ============================================================

void IRAM_ATTR readEncoder()
{
    int estadoB =
        digitalRead(ENC_B);


    portENTER_CRITICAL_ISR(
        &encoderMux
    );


    if (estadoB == LOW)
    {
        encoderCount -= ENCODER_SIGN;
    }
    else
    {
        encoderCount += ENCODER_SIGN;
    }


    portEXIT_CRITICAL_ISR(
        &encoderMux
    );
}


// ============================================================
// INICIALIZACION
// ============================================================

void encoderBegin()
{
    pinMode(
        ENC_A,
        INPUT
    );

    pinMode(
        ENC_B,
        INPUT
    );


    attachInterrupt(
        digitalPinToInterrupt(ENC_A),
        readEncoder,
        RISING
    );
}


// ============================================================
// LEER ENCODER
// ============================================================

long getEncoder()
{
    long count;


    portENTER_CRITICAL(
        &encoderMux
    );

    count =
        encoderCount;

    portEXIT_CRITICAL(
        &encoderMux
    );


    return count;
}


// ============================================================
// CERO ENCODER
// ============================================================

void zeroEncoder()
{
    portENTER_CRITICAL(
        &encoderMux
    );

    encoderCount = 0;

    portEXIT_CRITICAL(
        &encoderMux
    );
}


// ============================================================
// POSICION
// ============================================================

float getPositionMM()
{
    long count =
        getEncoder();

    return
        static_cast<float>(count)
        * MM_PER_COUNT;
}


// ============================================================
// MOSTRAR ESTADO
// ============================================================

void printEncoderStatus()
{
    long count =
        getEncoder();


    float revolutions =
        static_cast<float>(count)
        / COUNTS_PER_OUTPUT_REV;


    float angle =
        revolutions
        * 360.0f;


    float position =
        revolutions
        * LEAD_MM_REV;


    Serial.println();

    Serial.println(
        "# --------------------------------"
    );


    Serial.print(
        "# Encoder = "
    );

    Serial.println(
        count
    );


    Serial.print(
        "# Vueltas_salida = "
    );

    Serial.println(
        revolutions,
        4
    );


    Serial.print(
        "# Angulo_salida_deg = "
    );

    Serial.println(
        angle,
        2
    );


    Serial.print(
        "# Posicion_teorica_mm = "
    );

    Serial.println(
        position,
        4
    );


    Serial.println(
        "# --------------------------------"
    );

    Serial.println();
}
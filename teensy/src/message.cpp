/**
 * @file message.cpp
 * @brief Message source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#include <basketbuddy/message.h>
#include <basketbuddy/tasks.h>

Message *create_message(MessageType type, String data)
{
    return message_generation[type - TYPE_OFFSET](data);
}

Message *Message::parse(char *msg)
{
    if (msg[0] < TYPE_OFFSET || msg[0] > M_Telemetry)
        return nullptr;
    MessageType type = (MessageType)msg[0];
    uint8_t checksum = (uint8_t)msg[CMD_BUFFER_SIZE - 1];
    char data[30];
    strncpy(data, msg + 1, CMD_BUFFER_SIZE - 2);

    switch (type)
    {
    case M_Signal:
        return new SignalMessage(data, checksum);
    case M_Move:
        return new MoveMessage(data, checksum);
    case M_Lift:
        return new LiftMessage(data, checksum);
    case M_Log:
        return new LogMessage(data, checksum);
    default:
        return nullptr;
    }
}

// SignalMessage

int SignalMessage::process()
{
    switch ((uint8_t)data[0])
    {
    case S_Estop:
        send(S_Acknowledge);
        BasketBuddy::emergency_stop();
        break;
    case S_EstopRelease:
        noInterrupts();
        BasketBuddy::estop = ES_Disabled;
        BasketBuddy::robot_state = R_Startup;
        interrupts();
        send(S_Acknowledge);
        break;
    case S_Startup:
        if (BasketBuddy::robot_state != R_Ready)
            initialize();
        send(S_Startup);
        break;
    case S_Shutdown:
        send(S_Acknowledge);
        BasketBuddy::shutdown();
        break;
    case S_ErrorRelease:
        noInterrupts();
        BasketBuddy::robot_state = R_Startup;
        interrupts();
        break;
    case S_Error:
        // check broken
        if (strcmp(data.substring(1).c_str(), "broken pipe") == 0)
        {
            handle_broken_pipe();
        }

    default:
        send(S_Error);
        break;
    }
    return 0;
}

void SignalMessage::send(SignalType s)
{
    BasketBuddy::send(M_Signal, String((char)s));
}

void SignalMessage::send(SignalType s, String d)
{
    BasketBuddy::send(M_Signal, padString(String((char)s) + d, 30, '\0'));
}

// MoveMessage

MoveMessage::MoveMessage(String d) : Message(M_Move, d)
{
    phi_l = d.substring(0, d.indexOf(',')).toInt();
    d = d.substring(d.indexOf(',') + 1);
    phi_r = d.substring(0, d.indexOf(',')).toInt();
    d = d.substring(d.indexOf(',') + 1);
    duration = atoll(d.substring(0, d.indexOf(',')).c_str());
}

int MoveMessage::process()
{
    BasketBuddy::velocity.set(phi_l, phi_r, duration);
    return 0;
}

void MoveMessage::send(Velocity vel)
{
    BasketBuddy::send(M_Move, String(vel.left()) + "," + String(vel.right()) + "," + String(vel.countdown()));
}

void MoveMessage::send()
{
    MoveMessage::send(BasketBuddy::velocity);
}

// LiftMessage
LiftMessage::LiftMessage(String data) : Message(M_Lift, data)
{
    state = (LiftState)data.toInt();
}

int LiftMessage::process()
{
    int ret = false;
    if (state == LS_MovingDown)
        ret = async_lift_down();
    else if (state == LS_MovingUp)
        ret = async_lift_up();

    if (!ret)
    {
        SignalMessage::send(S_Error, "E_LIFT");
        return 0;
    }

    send();
    return 0;
}

void LiftMessage::send(Lift l)
{
    BasketBuddy::send(M_Lift, String(l.state));
}

void LiftMessage::send()
{
    noInterrupts();
    Lift l = {BasketBuddy::lift.position, BasketBuddy::lift.state};
    interrupts();
    LiftMessage::send(l);
}

// LogMessage

void LogMessage::send(String s)
{
    BasketBuddy::send(M_Log, padString(s, CMD_BUFFER_SIZE - 2));
}

// TelemetryMessage

void TelemetryMessage::send(String component, String data)
{
    BasketBuddy::send(M_Telemetry, padString(component, 10) + padString(data, 20));
}
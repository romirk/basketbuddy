/**
 * @file message.h
 * @brief Header file for the message module.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 *
 * This file contains the Message class and related definitions.
 */

#pragma once

#include <algorithm>
#include <numeric>
#include <string>
#include <basketbuddy/basketbuddy.h>
#include <basketbuddy/util.h>
#include <basketbuddy/lifting.h>

/**
 * @brief The Message class
 * Generic message class. All messages should inherit from this.
 *
 * Messages are sent over serial in the following format:
 * <command: 1 byte><data: null-padded 30 bytes><checksum: 1 byte>
 *
 * command is an unsigned integer corresponding to a MessageType.
 * data is a string of data that is specific to the message type.
 * checksum is a 1 byte checksum of the message.
 *
 * The checksum is calculated by summing the ASCII values of the message and taking the modulus 256.
 * See @ref validate for more details.
 */
class Message
{

public:
    const MessageType type; // The type of the message.
    String data = "";       // generic data field.
    uint8_t checksum = 0;   // The checksum of the message.

    Message(MessageType n) : type(n) {}
    Message(MessageType n, String d) : type(n), data(d) {}
    Message(MessageType n, String d, uint8_t c) : type(n), data(d), checksum(c) {}

    /**
     * @brief parse
     *
     * Parses a message from a string.
     */
    static Message *parse(char *msg);

    /**
     * @brief process
     *
     * Processes the message.
     *
     * @return int
     */
    virtual int process()
    {
        return 0;
    }

    /**
     * @brief checksum
     *
     * Calculates the checksum of the message.
     *
     * @return String
     */
    virtual uint8_t gen_checksum()
    {
        auto s = to_string();
        return std::accumulate(s.begin(), s.end(), 0) % 256;
    }

    /**
     * @brief validate
     *
     * Validates the message.
     *
     * @return true
     * @return false
     */
    virtual bool validate()
    {
        return (gen_checksum() == checksum);
    }

    /**
     * @brief to_string
     *
     * Converts the message to a string.
     *
     * @return String
     */
    virtual String to_string()
    {
        return String((char)type) + data;
    }

    /**
     * @brief serialize
     *
     * Serializes the message.
     *
     * @return String
     */
    virtual const char *serialize()
    {
        char *buffer = new char[CMD_BUFFER_SIZE];
        buffer[0] = (char)type;
        buffer[CMD_BUFFER_SIZE - 1] = (char)gen_checksum();
        strncpy(buffer + 1, data.c_str(), CMD_BUFFER_SIZE - 2);
        return buffer;
    }

    // virtual operator Message() const { return *this; }
};

class SignalMessage : public Message
{

    SignalType signal;

public:
    SignalMessage(SignalType s) : Message(M_Signal, String((char)s)), signal(s) {}
    SignalMessage(String s) : Message(M_Signal, s) {}
    SignalMessage(String s, uint8_t c) : Message(M_Signal, s, c) {}

    int process() override;

    static void send(SignalType);
    static void send(SignalType, String);
};

class MoveMessage : public Message
{
public:
    int phi_l;
    int phi_r;
    uint32_t duration;
    MoveMessage() : MoveMessage(0, 0, 0) {}
    MoveMessage(String d);
    MoveMessage(String d, uint8_t c) : MoveMessage(d) { checksum = c; };
    MoveMessage(int phi_l, int phi_r, uint32_t duration) : Message(M_Move, String(phi_l) + "," + String(phi_r) + "," + String(duration)), phi_l(phi_l), phi_r(phi_r), duration(duration) {}

    int process() override;

    static void send();
    static void send(Velocity);
};

class LiftMessage : public Message
{
    LiftState state;

public:
    LiftMessage() : LiftMessage(0, 0) {}
    LiftMessage(String d);
    LiftMessage(String d, uint8_t c) : LiftMessage(d) { checksum = c; }
    LiftMessage(LiftState l) : Message(M_Lift, String(l)) {}

    int process() override;

    static void send();
    static void send(Lift);
};

class LogMessage : public Message
{
public:
    LogMessage(String d) : Message(M_Log, d) {}
    LogMessage(String d, uint8_t c) : Message(M_Log, d, c) {}

    int process() override { return 0; };

    static void send(String);
};

class TelemetryMessage : public Message
{
public:
    TelemetryMessage(String d) : Message(M_Telemetry, d) {}
    TelemetryMessage(String d, uint8_t c) : Message(M_Telemetry, d, c) {}

    int process() override { return 0; };

    static void send(String, String);
};

typedef Message *(*Creator)(String);

template <typename T>
static Message *make(String data) { return new T(data); }

static Creator const message_generation[] = {make<LogMessage>, make<SignalMessage>, make<MoveMessage>, make<LiftMessage>, make<TelemetryMessage>};
Message *create_message(MessageType type, String data);

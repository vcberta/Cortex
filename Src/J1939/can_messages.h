#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

/** @file can_messages.h
 *  @brief This file contains the different structures of the CAN Messages
 *
 *
 *  @author Victor Berzosa (VCBERTA)
 */

typedef const struct
{
  int   id;             // ID of the Message
  int   extended;       // Is 29 bit identifier
  int   data[8];        // 8 Bytes of message;
  int   cycle_time;     // how often is the message sent;
}can_message_t;

typedef struct 
{
  can_message_t *       message;
  int                   last_cycle;
}can_item_t;


#endif //CAN_MESSAGES_H
#ifndef SCREENER_H
#define SCREENER_H

/** @file Screener.h
 *  @brief This file contains the database of the Screener project
 *  Only J1939 Messages will be sent in this example
 *
 *
 *  @author Victor Berzosa (VCBERTA)
 */
#include "can_messages.h"

   can_message_t message_screener_vep1 = {   
     0x18FEF7FE,        // ID of the Message
     1,              // Is 29 bit identifier      
     {0x7D, 0x0, 0xF0, 0x0, 0xF0, 0, 0xFC, 0}, // 8 Bytes of message;
     1000// how often is the message sent;
   };

  can_item_t  screener_list[] =
  {
    {&message_screener_vep1, 0}
  };

#endif //SCREENER_H
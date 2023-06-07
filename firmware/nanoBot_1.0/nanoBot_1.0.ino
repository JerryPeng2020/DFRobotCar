/****************************************************
 * Copyright (C) 2016-2023 Pinecone.AI - All Rights Reserved
 * 
 * Project: DFRobot firmwire Arduino Project
 * Version: 1.0
 * 
 * Date:   Jun. 6th, 2023
 * Author: Jerry Peng
 * Web:    www.pinecone.ai
 *  
/***************************************************/


#include "nanoBot.h"

nanoBot bot;

void setup()
{
  bot.init();
}



void loop()
{
  bot.ctrlLoop();
}

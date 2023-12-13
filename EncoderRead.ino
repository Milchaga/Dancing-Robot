// Calculate encoderA position when EncA_A changes
void EncA_Counter()
{
  if (digitalRead(EncA_A) == digitalRead(EncA_B))
  { 
    EncAPulse++;
  }
  else
  { 
    EncAPulse--;
  }
  
  /*
  Serial.println(EncAPulse);
  EncA_Pos = EncAPulse * CNVRT_pulses_to_meters;
  Serial.println((String)"Position A is:"+EncAPulse +" Pulses OR " + String(EncA_Pos,3) + "m");
  */
}

// Calculate encoderB position when EncB_A changes
void EncB_Counter()
{
  if (digitalRead(EncB_A) == digitalRead(EncB_B))
  { 
    EncBPulse--;
  }
  else
  { 
    EncBPulse++;
  }
  /*
  //Serial.println(EncBPulse);
  EncB_Pos = EncBPulse * CNVRT_pulses_to_meters;
  Serial.println((String)"Position B is:"+EncBPulse +" Pulses OR " + String(EncB_Pos,3) + "m");
  */
}

/*    !!! CHECK DIRECTION     */
   

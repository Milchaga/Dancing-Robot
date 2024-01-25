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
}

/*    !!! CHECK DIRECTION     */
   

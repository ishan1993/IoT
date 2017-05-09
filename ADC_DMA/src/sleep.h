#ifndef EM_SET
#define EM_SET EM0
#endif

#ifndef SRC_SLEEP_H_
#define SRC_SLEEP_H_


/*************************************************************/
/* Routine to put device in a predefined sleep mode          */
/* Input Variables - None									 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Sleep routine written based on example from 				 */
/* Professor Keith Graham's and silicon labs' example code 	 */
/*************************************************************/
void sleep(void);


/*************************************************************/
/* Routine to set a block for a sleep mode                   */
/* Input Variables - EM_State_local which is the mode needs	 */
/*					 to be blocked							 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Block Sleep routine written based on example from 		 */
/* Professor Keith Graham's and silicon labs' example code 	 */
/*************************************************************/
void Set_Sleep_Mode(Energy_Modes_Enum EM_State_local);


/*************************************************************/
/* Routine to unblock a sleep mode    		                 */
/* Input Variables - EM_State_local which is the mode needs	 */
/*					 to be blocked							 */
/* Global Variables - sleep_block_counter is used to track	 */
/*  				  which is the lowest mode, device can   */
/*					  go to sleep 							 */
/* Return Variables - None									 */
/* Unblock Sleep routine written based on example from 		 */
/* Professor Keith Graham's and silicon labs' example code 	 */
/*************************************************************/
void Clear_Sleep_Mode(Energy_Modes_Enum EM_State_local);



/* Routine to put device in a predefined sleep mode          */
inline void sleep(void)
{
       if(sleep_block_counter[EM0] > 0)
       {
              return;
       }
       else if(sleep_block_counter[EM1] > 0)
       {
    	   EMU_EnterEM1();
       }
       else if(sleep_block_counter[EM2] > 0)
       {
    	   EMU_EnterEM2(EM2_Restore);
       }
       else if(sleep_block_counter[EM3] > 0)
       {
    	   EMU_EnterEM3(EM3_Restore);
       }
       else
       {
    	   EMU_EnterEM3(EM3_Restore);
       }
       return;
}

/* Block sleep mode */
inline void Set_Sleep_Mode(Energy_Modes_Enum EM_State_local)
{
	INT_Disable();
	sleep_block_counter[EM_State_local]++;
	INT_Enable();
}

/* Unblock sleep mode */
inline void Clear_Sleep_Mode(Energy_Modes_Enum EM_State_local)
{
	INT_Disable();
	sleep_block_counter[EM_State_local]--;
	INT_Enable();
}

#endif

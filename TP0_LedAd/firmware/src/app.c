/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32DriverLcd.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
ST_BIT bitLeds;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    bitLeds.Flag = SET_FLAG;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // =====                   LCD                   ====
            // ==================================================
            // Initialisation du LCD avec activation du backlight
            lcd_init();
            lcd_bl_on();
            
            // Affichage du message initial
            printf_lcd("TP Led+AD 2022-23");
            lcd_gotoxy(1,2);
            printf_lcd("Farinas Einar");
            
            // =====                   ADC                   ====
            // ==================================================
            // Initialisation du  ADC
            BSP_InitADC10();
            // =====                   LEDs                  ====
            // ==================================================
            // Allumer toutes les LEDs
            ALL_LEDs(LEDS_ON);
            
            // =====                 Timer1                  ====
            // ==================================================
            // Démarrage du timer1
            DRV_TMR0_Start();
            
            // =====            chagement d'état             ====
            // ==================================================
            APP_UpdateState(APP_STATE_WAIT);
        }
        
        case APP_STATE_WAIT:
        {
            // Rien faire
            break;
        }
        
        case APP_STATE_SERVICE_TASKS:
        {
            // Eteindre toutes les LED une seule fois lors qu'on rentre dans
            // cette etat la première fois
            if(bitLeds.Flag)
            {
                ALL_LEDs(LEDS_OFF);
                bitLeds.Flag = RESET_FLAG;
            }
            // Lecture des deux potentiomètres et sauvegarde dans 
            // le struture AdcRes
            appData.AdcRes = BSP_ReadAllADC();
            
            // Affichage de deux valeurs lues
            lcd_gotoxy(1,3);
            printf_lcd("CH0: %4d  CH1: %4d", 
                    appData.AdcRes.Chan0, appData.AdcRes.Chan1);
            
            // Décalage des LEDs pour faire le chenillard
            Chenillard(RIGHT, 1);
            
            // Retour à l'état wait
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void APP_UpdateState(APP_STATES newState) 
{
    appData.state = newState;
}

void ALL_LEDs(uint8_t flag)
{
	// Si flag = 1
    if(flag)
    {	
		// Allumer toutes les LEDs (LED_0 à LED_7)
        BSP_LEDOn(BSP_LED_0);
        BSP_LEDOn(BSP_LED_1);
        BSP_LEDOn(BSP_LED_2);
        BSP_LEDOn(BSP_LED_3);
        BSP_LEDOn(BSP_LED_4);
        BSP_LEDOn(BSP_LED_5);
        BSP_LEDOn(BSP_LED_6);
        BSP_LEDOn(BSP_LED_7);
    }
    else
    {
		// Eteindre toutes les LEDs (LED_0 à LED_7)
        BSP_LEDOff(BSP_LED_0);
        BSP_LEDOff(BSP_LED_1);
        BSP_LEDOff(BSP_LED_2);
        BSP_LEDOff(BSP_LED_3);
        BSP_LEDOff(BSP_LED_4);
        BSP_LEDOff(BSP_LED_5);
        BSP_LEDOff(BSP_LED_6);
        BSP_LEDOff(BSP_LED_7);
    }
}

void Chenillard(uint8_t sens, uint8_t nb_Leds)
{
    // Liste des LEDs utilisés
    // Tableu avec les compteurs pour la sélection de position des LEDs du
    // chenillard
    // Case 0 pour la premièere LED, Case 1 pour la deuxième, etc.
    static List_Leds CntLeds[7] = {0,1,2,3,4,5,6};
    // Compteur pour la boucle for
    uint8_t i = 0;
    
    // Eteindre toutes les LEDs
    ALL_LEDs(LEDS_OFF);
    
    // Gestion des LEDs utilisé en fonction de nb_Leds
    for(i = 0; i < nb_Leds ; i++)
    {
        // Allumage des LEDs utilisé selon la valeur de son compteur
        // =========================================================
        switch(CntLeds[i])
        {
            case LED_0:
                BSP_LEDOn(BSP_LED_0);
                break;
            case LED_1:
                BSP_LEDOn(BSP_LED_1);
                break;
            case LED_2:
                BSP_LEDOn(BSP_LED_2);
                break;
            case LED_3:
                BSP_LEDOn(BSP_LED_3);
                break;
            case LED_4:
                BSP_LEDOn(BSP_LED_4);
                break;
            case LED_5:
                BSP_LEDOn(BSP_LED_5);
                break;
            case LED_6:
                BSP_LEDOn(BSP_LED_6);
                break;
            case LED_7:
                BSP_LEDOn(BSP_LED_7);
                break;
            default:
                // Si erreur allumer toutes les LEDs
                ALL_LEDs(LEDS_ON);
                break;
        }
        
        // Incrémentation ou décrémentation des compteurs afin de décaler leur
        // position
        // ===================================================================
        // Si on sélectione le sens gauche à droite
        if(sens == RIGHT)
        {
            // Incrémenter LEDs de 0 à 7 avec rébouclement
            if(CntLeds[i] < LED_7)
            {
                CntLeds[i]++;
            }
            else
            {
                CntLeds[i] = LED_0;
            }
        }
        // Si on sélectione le sens de droite à gauche
        else if (sens == LEFT)
        {
            // Décrémenter LEDs 7 à 0avec rébouclement
            if(CntLeds[i] > LED_0)
            {
                CntLeds[i]--;
            }
            else
            { 
                CntLeds[i] = LED_7;
            }
        }
    }
   
   
}

/*******************************************************************************
 End of File
 */

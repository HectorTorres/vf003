#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "wiringPiI2C.h"

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <math.h>

#define DEBUG_STATUS 1
#define DEBUG_STATUS_HIGH_SPEED 0

#define TIMER_DELAY 40
#define TIMER_PLOT 100
#define TIMER_SENSOR 40

#define TIME_ALARM 8000

#define RISE_TIME_COMPENSATOR 50

#define TRESHOLD_TA 7
#define PEEP_VAL 5
#define TRESHOLD_TD 15

#define PRESS_LIMIT_MAX 50

#define DEBOUNCE_TIME 500000

#define ValveExp 7

#define RG 2000
#define V0 1600
#define Vmax 26392
#define SLOPE_PRESSURE_SENSOR 0.26

#define AD_BASE 120

#define DATA 12
#define CLK 12
#define LATCH 12

#define REL1 9
#define REL2 11
#define REL3 5
#define REL4 10
#define REL5 6
#define REL6 13
#define REL7 19
#define REL8 1

#define RELE1 16
#define RELE2 20
#define RELE3 12
#define RELE4 21

#define AIR_PRESS_ALARM 4
#define O2_PRESS_ALARM 17

/**************************************************************************************************************************************/
//INICIO.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* BOTÓN DE INICIO */
void MainWindow::on_pushButton_start_clicked()
{
    if(timerStatusFlag){
        valvesMainControl(0);
        valvesExControl(1);
        timeFromInit=0;
        controlTimer->stop();
        sensorTimer->stop();
        plotTimer->stop();
        stopTimer->start(250);
        ui->pushButton_start->setText("Inicio.");
        timerStatusFlag=false;
        timeFromStart = 0;
        if(DEBUG_STATUS){   qDebug() << "Timer Stops."; }
        digitalWrite(RELE4,LOW);
        initVariables();
        on_pushButton_alarmTest_3_clicked(); // Activar botón de silenciar para evitar alarmas.
    }
    else {
            //validacion();
            evalVel();
            readedPressBottom = true;
            timeFRvTemporal = millis();
            timeMasterControl = millis();
            timeFromStart=millis();
            espontaneoPeriod = millis();
            inspirationTimeTop = false;  // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
            vavleChange=true;
            controlTimer->start(TIMER_DELAY);
            sensorTimer->start(TIMER_SENSOR);
            plotTimer->start(TIMER_PLOT);
            stopTimer->stop();
            ui->pushButton_start->setText("Paro.");
            timerStatusFlag=true;
            timeActiveVent = QDateTime::fromString("1 00:00:00","d HH:mm:ss");
            if(DEBUG_STATUS){   qDebug() << "Timer Starts."; }
            //valvesMainControl(0); // REVISARRRRRRRRRRRRRRRRRRRRRR para evitar el error de el inicio y/o primer inspiración.
        }
}

/**************************************************************************************************************************************/
//TEMPORAIZADOR DE SENSORES.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR TIMER FUNCTION
 * reads and checs the preassure sensor data */
void MainWindow::sensorTimerFunction(){

     readedPress=pressureRead();
    //Com en INN 8-18-2020
     readedPress = readedPress*1.1305-0.6738;


     if(readedPress>maxReadedPress){
        maxReadedPress=readedPress;
     }

     pressProm[0] = pressProm[1]; pressProm[1] = pressProm[2]; pressProm[2] = pressProm[3]; pressProm[3] = pressProm[4]; pressProm[4] = pressProm[5];  pressProm[5] = readedPress;
     double readedPressProm = (pressProm[0]+pressProm[1]+pressProm[2]+pressProm[3]+pressProm[4]+pressProm[5])/6.0;
     //readedPress = readedPressProm;

     if(ui->tabWidget_sel->currentIndex()==0){
        if(readedPress >= setPIP*1.05){
            valvesMainControl(0); //Cerrar valvulas
            valvesValueControl--;
            qDebug() << "Cerrando valvulas. Nuevo valor: "  << valvesValueControl;
        }
     }

     //Control de PEEP por valvula.
     if((inspirationTimeTop==true) && (readedPress <= (setPEEP+offsetPEEP))){
         slopeFilterTime = millis();
         valvesExControl(0x00);
         PEEPOnControl = true;
     }

     if(PEEPOnControl){
         if(indexPEEP<=250){
            PEEPData.replace(indexPEEP,readedPress);
            //qDebug() << PEEPData.size() << " index " << indexPEEP;

            indexPEEP++;
         }
     }

/* //Borrar una vez validadas las alarmas.
     if(ui->tabWidget_sel->currentIndex()==0){
        if(readedPress >= maxPressLimit){
            //valvesMainControl(0x00);
            //valvesExControl(0x01);
            activateAlarm(1);
            qDebug() << "Alarma de activación alta.";
        }
     }*/
/*
     if(readedPress <= 1 && ((millis()-timeFromStart) >= 3000)){
        valvesMainControl(0x00);
        valvesExControl(0x01);
        activateAlarm(2);
        qDebug() << "Alarma de activación baja.";
     }*/
/*
    if((readedPress/readedPressTempD) >= 4 || (readedPress/readedPressTempD) <= 0.1 ){
        readedPress=readedPressTempD;
    }*/

    readedPressTempD = readedPress;

    //Pendiente--------------------------------------------------------
    if(indexPress<=4){
        pressSlope.replace(indexPress,readedPressProm);
    }
    else {
        if((millis()-slopeFilterTime)>=400){
            std::rotate(pressSlope.begin(),pressSlope.begin()+1,pressSlope.end());
            pressSlope.replace(4,readedPressProm);
        }
    }
   double slope=0;
   slope=std::accumulate(pressSlope.begin(),pressSlope.end(),slope);
   slope=atan(((pressSlope[0]-pressSlope[1])+(pressSlope[1]-pressSlope[2])+(pressSlope[2]-pressSlope[3])+(pressSlope[3]-pressSlope[4]))/5)*180/3.1415;
   //qDebug() << "Slope: " << slope;

   if((ui->radioButton_assit->isChecked() || ui->radioButton_esp->isChecked()) && (millis()-timeInPeak)>=(periodTime/7.0)){
        if(slope >= double(ui->horizontalSlider_sensibilidad->value())*6.1111+23.889){ // f001 Evaluación de el nuevo valor de la pendiente.
            qDebug() << "Slope angle inspiratio:" <<slope;
            timeMasterControl+=double(60.0/double(FRv))*1000;
            inspirationDetected = true;
            inspirationInitTime = millis();
        }
   }

    if(slopeIncrease == true && (millis()-timerSlopeIncrease) >= timeFRv){
        slopeIncrease = false;
    }

    if(!ui->radioButton_esp->isChecked()){
        if(ui->checkBox_alarmNoConnection->isChecked()){
            //qDebug() << "sLOPE: " << slope << "  slopeFlat: " << slopeFlat;
            if(slope<=0.2 && slope>=-0.2 && slopeFlat == false){
                qDebug() << "Deteccion de desconexión";
                timeFlatSlope = millis();
                slopeFlat=true;
        }

    //Evitar errores en alarma de desconexión
        if(((slope>=1) || (slope<=-1)) && (slopeFlat == true)){
            slopeFlat = false;
        }

   //if(slope<=0.2 && slope>=-0.2 && slopeFlat == true && (millis()-timeFlatSlope)>6000){
   if(slope<=0.2 && slope>=-0.2 && slopeFlat == true && (millis()-timeFlatSlope)>(((60/FRv)*1.5)*1000)){
       if(flagAlarmDesc == false){
       qDebug() << "Deteccion de desconexión confirmada";
       estadoDesconexion = " | Desconectado!";
       noConection = true;
       flagAlarmDesc = true;
       alarmOn = true;
       alarmText = " PEEP bajo o desconexion del paciente. |";
       //ui->label_estado->setText(ui->label_estado->text() + " | DesconexiÓn.");
       timeFlatSlope = 0;
       //activateAlarm(9);
       slopeFlat=false;
       }
    }

   //Re-activar la variable de control de PEEP offset cuando vuelva a haber presión.
   if(readedPress>=2.5){
       noConection = false;
   }

   if((millis()-timeFlatSlope)>10000){
       slopeFlat=false;
    }
    }
    }

    //Evitar errores en la lectura del sensor de flujo
   readedFlow =(flowRead(initI2C));
   if((readedFlow>=0) && (readedFlow<=2000)){
   }
   else {
       readedFlow = 0;
   }
   //qDebug() << readedFlow;

   flowProm[0] = flowProm[1]; flowProm[1] = flowProm[2]; flowProm[2] = flowProm[3]; flowProm[3] = flowProm[4]; flowProm[4] = flowProm[5];  flowProm[5] = readedFlow;
   double readedFlowProm = (flowProm[0]+flowProm[1]+flowProm[2]+flowProm[3]+flowProm[4]+flowProm[5])/6.0;
   readedFlow = readedFlowProm;

   if(flowDataReadStatus==false){
        volTemp=0;
    }

   if(readedFlow>=maxFlow){
    maxFlow = readedFlow;
   }

    //readedVol = volRead(readedFlow)*1.13;
   //Compensador de volumen para diferentes PEEPs
   if(setPEEP >= 4 && setPEEP <= 11){
        readedVol = volRead(readedFlow)*1.13*(1/((-0.02*setPEEP)+1.15));
    }
   else{
        readedVol = volRead(readedFlow)*1.13*(1/((-0.02*11)+1.15));
   }

   //Compensador de volumen para diferentes distensibilidades.
    //readedVol = readedVol*(-0.0033*maxReadedPress+1.0478);
    //readedVol = readedVol*(-0.0049*maxReadedPress+1.0968);
   readedVol = readedVol*(-0.0058*maxReadedPress+1.1308);


    //Almacenar los datos en los vectores_______________________________________________
    if(indexPress<=251){
    pressData.replace(indexPress,readedPress);
    flowData.replace(indexPress,readedFlow);
    volData.replace(indexPress,readedVol);
    indexPress++;
    }
    if(indexPress>=251) {
        indexPress=0;
    }

    //Alarma de presion_________________________________________________________
    ui->label_press_5->setText(QString::number(readedPressProm,'f',1));
    if((readedPress >= maxPressLimit)  &&  (flagAlarmPresMax == false)){
        flagAlarmPresMax = true;
        alarmOn = true;
        alarmText+= " Presión alta.|";
    }
}

/**************************************************************************************************************************************/
//TEMPORAIZADOR DE CONTROL PRINCIPAL.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::controlTimerFunction(){

        periodTime = double(60.0/double(FRv))*1000;
        inspirationTime = periodTime/(ieRatioRef+1);

        //if(ui->radioButton_esp->isChecked())

        //Inspiracion mandatoria y asistida
        if(!ui->radioButton_esp->isChecked()){

        if(((millis()-timeMasterControl) >= uint32_t(inspirationTime)) && (inspirationTimeTop == false)){
            timeInPeak = millis();
            timeInPeakRead = millis() - slopeFilterTime;
            qDebug() << "Time up: " << timeInPeak;
            valvesMainControl(0x00);
            delayMicroseconds(10000);
            valvesExControl(0x01);
            inspirationTimeTop = true;
            flowDataReadStatus = false;

        // Sistema de control básico./////////////////////////////////////////////////////
         if(ui->checkBox_control->isChecked()){
             if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive==true))){

                 int errorPress = int((setPIP-readedPress)*Kp);

                 if(((readedPress)<=(setPIP-((double(lowPressPorc)/100.0)*setPIP))) && (flagAlarmPressLos == false)){
                     pressLowAlarm++;
                     if(pressLowAlarm>=8){
                          alarmOn = true;
                          flagAlarmPressLos = true;
                          alarmText = " PIP bajo. |";
                     }
                 }
                 qDebug() << "$$$ Alarma de presion: " <<pressLowAlarm << " Rango: " << (readedPress) << "<="<< setPIP-((double(lowPressPorc)/100.0)*setPIP);
                 //qDebug() << "Compensador: " << errorPress;
                 if(currentMaxPress <= setPEEP+2) {errorPress = 0;}
                 valvesValueControl+=errorPress; //Validar por conversion implicita
                 //qDebug() << "Valvulas: " << valvesValueControl;
            }
             if(ui->tabWidget_sel->currentIndex()==1 || ((ui->tabWidget_sel->currentIndex()==2) && (volControlActive==true))){
                 if(readedVol<=setVOL){
                     valvesValueControl++;
                     if(valvesValueControl>=255){
                     valvesValueControl=255;}
                 }
                 else {
                     valvesValueControl--;

                 }

             }
          }

        ui->label_vti->setText(QString::number(readedVol,'f',1));
        volVTI = readedVol;

        if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive==true))){
            pressError = ((readedPress-setPIP)/setPIP)*100.0;
            if(pressError > 5 || pressError < -5){
                alarmControl= true;
                //digitalWrite(RELE4,HIGH);
                ui->label_ajuste->setText("Ajustando.");
                //ui->label_ajuste->setStyleSheet("color: rgb(170, 0, 0)");
                ui->label_ajuste->setStyleSheet("font-weight: 800; color: rgb(170, 0, 0)");
                //ui->label_ajuste->setStyleSheet("font: bold 11pt \"Sans Serif\";");
                if(pressError > 2 || pressError < -2){
                    ui->checkBox_control->setChecked(true);
                }
            }
            if(pressError < 5 && pressError > -5){
                alarmControl= false;
                pressLowAlarm=0;
                ui->label_ajuste->setText("Estable.");
                //ui->label_ajuste->setStyleSheet("color: rgb(0, 85, 0)");
                ui->label_ajuste->setStyleSheet("font-weight: 400; color: rgb(0, 85, 0)");
                //ui->label_ajuste->setStyleSheet("font: 11pt \"Sans Serif\";");
                if(pressError < 2 && pressError > -2){
                    ui->checkBox_control->setChecked(false);
                }
            }
            currentMaxPress = readedPress;
            ui->label_press_5->setText(QString::number(currentMaxPress,'f',1));
            qDebug() << "Presión máxima: " << readedPress << " Presion Objetivo: " << setPIP << " Porcentaje de diferencia: " << pressError << "%   Valvula: " << valvesValueControl;
            qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
        }
        if(ui->tabWidget_sel->currentIndex()==1 || ((ui->tabWidget_sel->currentIndex()==2) && (volControlActive==true))){
            volError = ((readedVol-setVOL)/setVOL)*100.0;
            if(volError > 8 || volError < -8){
                //digitalWrite(RELE4,HIGH);
                //ui->checkBox_control->setChecked(true);
                ui->label_ajuste->setText("Ajustando.");
                ui->label_ajuste->setStyleSheet("color: rgb(170, 0, 0)");
                if(volError > 1 || volError < -1){
                    ui->checkBox_control->setChecked(true);
                }
            }
            if(volError <= 8 && volError >= -8){
                //ui->checkBox_control->setChecked(false);
                ui->label_ajuste->setText("Estable.");
                ui->label_ajuste->setStyleSheet("color: rgb(0, 85, 0)");
                if(volError <= 1 && volError >= -1){
                    ui->checkBox_control->setChecked(false);
                }
            }
            qDebug() << "Volumen máxima: " << readedVol << " Vol Objetivo: " << setVOL << " Porcentaje de diferencia: " << volError << "%   Valvula: " << valvesValueControl;
            qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
        }
        }

/* //Borrar al validar las alarmas.
        if(millis()-timeInPeak >= 500 && ((alarmPressInputO2 == false) && (alarmPressInput == false))){
             // Desactivar audio
        }*/
/*
        if(ui->radioButton_esp->isChecked()){
            ui->tabWidget_sel->setTabEnabled(1,false);
        }
        else{
            ui->tabWidget_sel->setTabEnabled(1,true);
        }
*/
        //Fin de periodo////////////////////////////////////////////
        if(((millis()-timeMasterControl)>=periodTime) && (inspirationTimeTop == true)){
            ui->label_press_maxPress->setText(QString::number(maxReadedPress,'f',1));
            maxReadedPress = 0;
            slopeFilterTime = millis();
            qDebug() << "Period: " << millis();
            PEEPOnControl = false;

            if(indexPEEP >= 4){
            PEEPaverage = std::accumulate(PEEPData.begin()+3, PEEPData.begin()+indexPEEP, PEEPaverage);
            //qDebug() << PEEPData << " Index: " << indexPEEP;
            PEEPaverage = (PEEPaverage/(indexPEEP+1-3));
            PEEPaverage  = 1.124*PEEPaverage -1.0792;

            if((PEEPaverage >= setPEEP+0.1) || (PEEPaverage <= setPEEP+0.1)){
                offsetPEEP+=(setPEEP-PEEPaverage)*0.3;
                    if(offsetPEEP >= 4){
                        offsetPEEP = 4;
                    }
                    if(offsetPEEP <= -4){
                        offsetPEEP = -4;
                    }
            }

            if(setPEEPr >= setPEEP){
                setPEEP += (setPEEPr-setPEEP)*0.3;
                qDebug() << "______________________PEEPr: " << setPEEPr << " PEEP: " << setPEEP;
            }
            else{
                setPEEP=setPEEPr;
            }

            //qDebug() << PEEPData << " Index: " << indexPEEP;

            if(noConection){ // Esto para evitar un sobrecontrol cuando e desconecte el pulmón. Codigo 654825
                offsetPEEP=0;
            }

            indexPEEP = 0;
            qDebug() << "PEEP Value: "<< PEEPaverage << " PEEP Diff: " << (setPEEP-PEEPaverage) << " PEEP Offset: " << offsetPEEP;
            ui->label_PEEP->setText(QString::number(PEEPaverage,'f',1));
            PEEPaverage = 0;
            }
            else{
                PEEPaverage = 0;
                indexPEEP = 0;
                offsetPEEP = -2;
            }

            cicleCounter++;
            ui->label_ciclos->setText(QString::number(cicleCounter));



            periodTimeRead = (millis()-timeMasterControl);

            //qDebug() << "Tiempo inspiratorio: " << double(timeInPeakRead)/1000.0;
            //qDebug() << "Periodo en segundos: " << double(periodTimeRead)/1000.0;
            ui->label_period->setText(QString::number(double(periodTimeRead)/1000.0,'f',2));
            ui->label_ta->setText(QString::number(double(timeInPeakRead)/1000.0,'f',2));
            ui->label_tb->setText(QString::number(double(periodTimeRead-timeInPeakRead)/1000.0,'f',2));

            //Alarmas///////////////////////////////////////////////
            /*----------------------------------------------------*/


            //Alarma de flujo________________________________________________
            //if(!inspirationTimeTop && (volControlActive== true)){
            if(volControlActive== true){

            double flowLm = (setVOL/(1000.0))*FRv;
            ui->label_volSetmin->setText(QString::number(flowLm-flowLm*(flowError/100),'f',1));
            ui->label_volSetmax->setText(QString::number(flowLm+flowLm*(flowError/100),'f',1));
            ui->label_flowAlarm->setText(QString::number((volVTI/1000.0)*frCurrent,'f',1)); //Conversión de ml/s a l/min
            ui->label_flowAlarm_2->setText(QString::number((volVTI/1000.0)*frCurrent,'f',1)); //Conversión de ml/s a l/min "0002"
            double volMinActual = (volVTI/1000.0)*frCurrent;
            qDebug() << "__________o_____________: " << readedVol << frCurrent;
            if(((cicleCounter-alarmCurrenCiclosVol)>=4) && ((cicleCounter-alarmCurrenCiclosFR)>=6)){
              if((millis()-inspirationInitTime)>=(inspirationTime*0.3)){
                if((volMinActual >= flowLm+flowLm*(flowError/100))  && (flagAlarmVolMinMax  == false)){
                    vmCounter++;
                    if(vmCounter>=2){
                        flagAlarmVolMinMax = true;
                        alarmOn = true;
                        alarmText+= " Vol*min alto.|";
                    }
                }
                if((volMinActual <= flowLm-flowLm*(flowError/100))  && (flagAlarmVolMinMin   == false)){
                    vmCounter++;
                    if(vmCounter>=2){
                        flagAlarmVolMinMin = true;
                        alarmOn = true;
                        alarmText+= " Vol*min bajo.|";
                    }
                }
              }
            }
            }

            ui->label_flowAlarm->setText(QString::number((volVTI/1000.0)*frCurrent,'f',1)); //Conversión de ml/s a l/min
            ui->label_flowAlarm_2->setText(QString::number((volVTI/1000.0)*frCurrent,'f',1)); //Conversión de ml/s a l/min "0002"

            maxFlow=0;


            /*----------------------------------------------------*/
            ui->label_fr_current->setText(QString::number((1000.0/double(millis()-timeMasterControl))*60.0,'f',1));
            ui->label_fr_current2->setText(QString::number((1000.0/double(millis()-timeMasterControl))*60.0,'f',1));

            //Alarma de fr____________________________
            frCurrent = (1000.0/double(millis()-timeMasterControl))*60.0;
            //qDebug() << "FR: " << frCurrent << "  menor que: " << (1000.0/double(millis()-timeMasterControl))*60.0;
            //ui->label_fr_current2->setText(QString::number(frCurrent,'f',1));
            ui->label_frSetmax->setText(QString::number(FRv+FRv*(frError/100),'f',1));
            ui->label_frSetmin->setText(QString::number(FRv-FRv*(frError/100),'f',1));

            if(ui->radioButton_manda->isChecked()){

            if((cicleCounter-alarmCurrenCiclosFR)>=4 && (cicleCounter-alarmCurrenCiclosVol)>=4){
                if((frCurrent >= FRv+FRv*(frError/100.0)) && (flagAlarmFRMax == false)){
                    frCounter++;
                    qDebug() << "Alarma FR alta";
                    if(frCounter>=2){
                        flagAlarmFRMax = true;
                        alarmOn = true;
                        alarmText+= " FR alto.|";
                    }
                }
                if((frCurrent <= FRv-FRv*(frError/100.0)) && (flagAlarmFRMin == false)){
                    frCounter++;
                    qDebug() << "Alarma FR baja";
                    if(frCounter>=2){
                        flagAlarmFRMin = true;
                        alarmOn = true;
                        alarmText+= " FR bajo.|";
                    }
                }
            }
            }



            ui->label_press->setText("1:" + QString::number(ieRatioRef));
            timeMasterControl = millis();
            valvesExControl(0x00);
            delayMicroseconds(10000);
            valvesMainControl(uint8_t(valvesValueControl));
            inspirationTimeTop = false;
            flowDataReadStatus = true;
            }
        }
        
        //Inspiracion espontanea
        if(ui->radioButton_esp->isChecked()){

            if(inspirationDetected){
                espontaneoPeriod=millis();
                cicleCounter++;
                ui->label_ciclos->setText(QString::number(cicleCounter));
                
                valvesExControl(0x00);
                delayMicroseconds(10000);
                valvesMainControl(uint8_t(valvesValueControl));
                inspirationDetected = false;
                inspirationDetected2 = true;
                flowDataReadStatus = true;
            }           
            if((millis()-espontaneoPeriod) >= uint32_t(inspirationTime) && inspirationDetected2==true){
                flowDataReadStatus = false;
                valvesMainControl(0x00);
                delayMicroseconds(10000);
                valvesExControl(0x01);
                inspirationDetected2 = false;
                //ui->label_press_maxPress->setText(QString::number(readedPress,'f',2)); // f001 Impresion de presion maxima
                ui->label_fr_current->setText("NA");
                ui->label_press->setText("1:" + QString::number(ieRatioRef));
            }
            if((millis()-espontaneoPeriod)>=15000){
                AlarmOut();
                alarmText+="Apnea. |";
                alarmOn = true;
                inspirationDetected=true;
            }
        }

        //QString alarmPressInputState = "Estado: ";
        //qDebug() << "Pin AIR: " << digitalRead(AIR_PRESS_ALARM) << "  Pin O2: " <<  digitalRead(O2_PRESS_ALARM);
        //Alarmas de presión de entrada
       /* if(ui->checkBox_alarmAirPress->isChecked()){
            if(digitalRead(AIR_PRESS_ALARM)  && (alarmOn == false)){
                alarmOn = true;
                alarmText+= " P.Aire baja.|";
            }
        }*/
      /*  if(ui->checkBox_alarmO2Press->isChecked()){
            if(digitalRead(O2_PRESS_ALARM)  && (alarmOn == false)){
               alarmOn = true;
               alarmText+= " P.O2 baja.|";
            }
        }*/


        alarmTimeRunn = millis();
        if((flagAlarmPresMax == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmPresMax = false;silence = false;}
        if((flagAlarmPressLos == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmPressLos = false;silence = false;}
        if((flagAlarmVolMinMax == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmVolMinMax = false;silence = false;}
        if((flagAlarmVolMinMin == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmVolMinMin = false;silence = false;}
        if((flagAlarmFiO2Max == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmFiO2Max = false;silence = false;}
        if((flagAlarmFiO2Min == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmFiO2Min = false;silence = false;}
        if((flagAlarmFRMax == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmFRMax = false;silence = false;}
        if((flagAlarmFRMin == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmFRMin = false;silence = false;}
        if((flagAlarmDesc == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmDesc = false;silence = false;}
        if((flagAlarmAirPres == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmAirPres = false;silence = false;}
        if((flagAlarmO2Pres == true) && (silence == true) && ((alarmTimeRunn-alarmTimeStop)>=TIME_ALARM)){
                flagAlarmO2Pres = false;silence = false;}


   //     if((alarmOn == true) && ((millis()-alarmTimeStop)>=5000)){ // Modificar tiempo de apagado de alarma 0001
        if(alarmOn == true){
            if((
                (flagAlarmPresMax == true)      ||
                (flagAlarmPressLos == true)     ||
                (flagAlarmVolMinMax == true)    ||
                (flagAlarmVolMinMin == true)    ||
                (flagAlarmFiO2Max == true)      ||
                (flagAlarmFiO2Min == true)      ||
                (flagAlarmFRMax == true)        ||
                (flagAlarmFRMin == true)        ||
                (flagAlarmDesc == true)         ||
                (flagAlarmAirPres == true)      ||
                (flagAlarmO2Pres == true))
                ){
            digitalWrite(RELE4,HIGH);
            silence = false;
            qDebug() << "ALARMA ACTIVADA!";
            ui->label_estado->setStyleSheet("color: rgb(170, 0, 0)");
            //ui->label_ajuste->setStyleSheet("font-weight: 800; color: rgb(170, 0, 0)");
            ui->label_estado->setText("Estado: " + alarmText);
            }
        }






}

/**************************************************************************************************************************************/
//LECTURA DE SENSORES.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE PRESIÓN */
double MainWindow::pressureRead(){
    uint16_t bitsA0 = uint16_t(analogRead(AD_BASE));
    //qDebug() << "-------------------------" << bitsA0;
    //double kPaSensor = (double(bitsA0));
    double kPaSensor = 0.0033*(double(bitsA0))*1.08*0.96*1.07*0.86 - 7.6136+2.6-0.9+2.19;
    kPaSensor = (kPaSensor*1.055);
    kPaSensor = (kPaSensor*0.9483);

    kPaSensor = (kPaSensor*slopePressureAdj) + offsetPressureAdj;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de presión: " << kPaSensor << " mmH2O." ; }
    return kPaSensor;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE FLUJO */
double MainWindow::flowRead(int I2Cfn){
    int flowRaw = wiringPiI2CReadReg16(I2Cfn,0x00);
    dataH = uint8_t(flowRaw);
    dataL = uint8_t(flowRaw>>8);
    dataFull = uint16_t(dataH<<8)+dataL;
    double flowSensor = (50.0*((double(dataFull)/16384.0)-0.1)/0.8)*16.67;
    flowSensor = flowSensor*(-0.0196647*log(flowSensor)+1.246406);
    flowSensor = (flowSensor*slopeFlowAdj)+offsetPressureAdj;
    //flowSensor = flowSensor*(ui->horizontalSlider_flowSlope->value())/100.0 + (ui->horizontalSlider_flowOffset->value()/20.0);
    //qDebug() << "I2C data: " << (dataFull) << " Flow: " << 50.0*((double(dataFull)/16384.0)-0.1)/0.8;
    //uint16_t bitsD23 = uint16_t(analogRead(AD_BASE+3));
    //double flowSensor = 0.00267694*(double(bitsD23))-3.07741234;
    //if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de flujo: " << flowSensor << " ml/min." ; }
    return flowSensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE VOLUMEN */
double MainWindow::volRead(double flowIn){
    double volTemp2 = (flowIn*(TIMER_SENSOR/1000.0))*1.15;
    volTemp = volTemp+volTemp2;
   // qDebug() << "Volumen: " << volTemp << " ml." << "   VolTemp2 " << volTemp2 << " Flag: " << flowDataReadStatus;
    return volTemp*1;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR DE O2 */  //Max 7500 O2  // Max aire 1500
double MainWindow::o2Read(){
    uint16_t bitsA1 = uint16_t(analogRead(AD_BASE+3));
    //qDebug() << "Bits O2: " << bitsA1;
    //double o2Sensor = double(bitsA1)*0.012625418 + 0.309364;  // GDL
    double o2Sensor = ((double(bitsA1)*0.0155 + 3.9887)*1.037 - 2.5862-4.8+0.4+2)*0.92;    // CDMX
    //qDebug() << "Bits A1: " << bitsA1 << " | Concentración: " << o2Sensor;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Saturación de O2: " << o2Sensor << " %." ; }
    return o2Sensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* MIN PRESSURE REGULADORES */
double MainWindow::pressSairRead(){
    uint16_t pressSair = uint16_t(analogRead(AD_BASE+2));
     return double(pressSair);
}

double MainWindow::pressSo2Read(){
    uint16_t pressSo2 = uint16_t(analogRead(AD_BASE+1));
     return double(pressSo2);
}


/**************************************************************************************************************************************/
//REGISTRO DE TIEMPO ACTIVO DEL SISTEMA. Y ALARMA DE INOPERABLE
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* TEMPORIZADOR PARA LLEVAR REGISTRO */
void MainWindow::activeTimerFunction(){

        double readPressureAirAlarm = pressSairRead();
        double readPressureO2Alarm = pressSo2Read();

        //qDebug() << "Aire:" <<readPressureAirAlarm << " Set 1266.";
        //qDebug() << "O2:" << readPressureO2Alarm << " Set 13300";

        if(ui->checkBox_alarmAirPress->isChecked()){
            if((readPressureAirAlarm <= 12600) && (flagAlarmAirPres == false)){
                flagAlarmAirPres = true;
                alarmOn = true;
                alarmText = " P. aire baja. |";
            }
        }
    /*    if(readPressureAirAlarm >= 50){
            alarmOn = true;
            alarmText = " P. aire alta. |";
        }*/
        if(ui->checkBox_alarmO2Press->isChecked()){

            if((readPressureO2Alarm <= 13300)  && (flagAlarmO2Pres == false)){
                flagAlarmO2Pres = true;
                alarmOn = true;
                alarmText = " P. O2 baja. |";
            }
        }
  /*      if(readPressureO2Alarm >= 50){
            alarmOn = true;
            alarmText = " P. O2 alta. |";
        }*/


timeActive = timeActive.addSecs(1);
ui->label_activeTime->setText(timeActive.toString("d HH:mm:ss"));
timeActiveToSave++;
if(timeActiveToSave>=10){
    timeActiveToSave=0;
    QFile file("/home/pi/TimeActive.txt");
    if(file.open(QFile::WriteOnly | QFile::Text)){
    }
    QTextStream dataToFile(&file);
    dataToFile << timeActive.toString("d HH:mm:ss");
    file.flush();
    file.close();
    }

if(timerStatusFlag){ // Incremento de tiempo de ventilación "0004"
    timeActiveVent = timeActiveVent.addSecs(1);
    ui->label_activeTimeVent->setText(timeActiveVent.toString("d HH:mm:ss"));
    }

if((readedPress <= -5.0) || (readedPress >= 101.0) || (readedFlow <= -5.0) || (readedPress >= 2000.0)){
    ventMainAlarmSystem = true;
    MainAlarmSystemTimer = millis();
        qDebug() << "Alarmá! sistema inoperable advertencia!.";
}

if(((readedPress <= -5.0) || (readedPress >= 101.0) || (readedFlow <= -5.0) || (readedPress >= 2000.0)) && (ventMainAlarmSystem == true) && ((millis() - MainAlarmSystemTimer)>=3000)){
    ventMainAlarmSystem = false;
    on_pushButton_start_clicked();
    QMessageBox::critical(this,"Error!.","Ventilador inoperable!.","Aceptar.");
}




}
/**************************************************************************************************************************************/
//CONTROL DE GRAFICAS DEL SISTEMA y ETAPA DE O2.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* PLOT FUNCTION
 * Plots data, check the timer starts from this function to see
 * which is the timeout set */
void MainWindow::plotTimerFunction(){
    plotData(ui->customPlot);
    ui->customPlot->replot();

    readedO2 = o2Read();

    O2Prom[0] = O2Prom[1]; O2Prom[1] = O2Prom[2]; O2Prom[2] = O2Prom[3]; O2Prom[3] = O2Prom[4]; O2Prom[4] = O2Prom[5]; O2Prom[5] = readedO2;
    double readedO2Prom = (O2Prom[0] + O2Prom[1] + O2Prom[2] + O2Prom[3] + O2Prom[4] + O2Prom[5])/6.0;
    readedO2 = readedO2Prom;

    if(readedO2>=105){
        readedO2 = 105;
    }

    ui->label_o2->setText(QString::number(readedO2,'f',0));
    ui->label_currentFio2->setText(QString::number(readedO2,'f',1));


    /*Alarma de FIO2 -------------------*/

    ui->label_fioSetmin->setText(QString::number(fioSetPoint-fioSetPoint*(fio2Error/100.0),'f',1));
    ui->label_fioSetmax->setText(QString::number(fioSetPoint+fioSetPoint*(fio2Error/100.0),'f',1));


    if(cicleCounter>=2){
    if((readedO2 > fioSetPoint+fioSetPoint*(fio2Error/100.0)) && (flagAlarmFiO2Max == false)){
        fio2Counter++;
            if(fio2Counter>=3){
                flagAlarmFiO2Max = true;
                alarmOn = true;
                alarmText = " FiO2 alto. |";
        }
    }

    if((readedO2 < fioSetPoint-fioSetPoint*(fio2Error/100.0)) && (flagAlarmFiO2Min  == false)) {
        fio2Counter++;
            if(fio2Counter>=3){
                flagAlarmFiO2Min = true;
                alarmOn = true;
                alarmText = " FiO2 bajo. |";
        }
    }
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* CONFIGURACIÓN DEL PLOT*/
void MainWindow::plotSetup(QCustomPlot *customPlot){
    x.insert(0,251,0.0);
    for (int i=0; i<251; ++i)
    {
      x.replace(i,i*TIMER_SENSOR/1000.0);
    }
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::darkGreen)); // line color red for second graph
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::red)); // line color red for second graph

    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setRange(0,251);
    customPlot->yAxis->setRange(-3,40);
    customPlot->yAxis2->setRange(-3,40);
    customPlot->xAxis->setLabel("Tiempo [s]");
    customPlot->yAxis->setLabel("Pr [cmH2O] / Fl [l/min] / Vol [ml]");

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    customPlot->xAxis->setTicker(timeTicker);

    customPlot->xAxis->setRange(0,251*TIMER_SENSOR/1000.0);
    timeTicker->setTimeFormat("%z");

    QSharedPointer<QCPAxisTickerFixed> fixedTicker(new QCPAxisTickerFixed);
    customPlot->xAxis->setTicker(fixedTicker);
    fixedTicker->setTickStep(1.0);
    fixedTicker->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
    customPlot->yAxis->setTicker(fixedTickerY);
    fixedTickerY->setTickStep(3);
    fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    //customPlot->xAxis->setTickLabelRotation(45);
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* GRAFICAR DATOS */
void MainWindow::plotData(QCustomPlot *customPlot)
{
  customPlot->graph(0)->setData(x, pressData);
  customPlot->graph(1)->setData(x, flowData);
  customPlot->graph(2)->setData(x, volData);

  minPEEP = *std::min_element(pressData.begin(), pressData.end())+0.2;
  if(minPEEP <= 1){
      minPEEP = 5.2;
  }



  if(ui->tabWidget_sel->currentIndex()==0 ){
        customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
        customPlot->graph(2)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
        customPlot->yAxis->setRange(-1,setPIP+6);
        customPlot->yAxis2->setRange(-1,setPIP+2);
        QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
        customPlot->yAxis->setTicker(fixedTickerY);
        fixedTickerY->setTickStep(2);
        fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
        customPlot->yAxis->setLabel("Presion [cmH2O]");
  }

  if(ui->tabWidget_sel->currentIndex()==1 ){
      customPlot->graph(2)->setBrush(QBrush(QColor(255, 0, 0, 20))); // first graph will be filled with translucent blue
      customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
      customPlot->yAxis->setRange(-3,setVOL+100);
      customPlot->yAxis2->setRange(-3,setVOL+100);
      QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
      customPlot->yAxis->setTicker(fixedTickerY);
      fixedTickerY->setTickStep(100);
      fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
      customPlot->yAxis->setLabel("Flujo [ml/s] / Volumen [ml]");
  }
  //         ui->label_PEEP->setText(QString::number(minPEEP,'g',3));
}

/**************************************************************************************************************************************/
//ACTIVACIÓN DE ALARMAS.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* ALARMAS */
void MainWindow::activateAlarm(uint16_t number){
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << number; }
    errorFrCounter=0;
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << errorFrCounter; }
    //on_pushButton_start_clicked();
    //evalVel();
    AlarmOut();
    digitalWrite(RELE4,HIGH);
    switch (number) {
    case 1: {
       // QMessageBox::critical(this,"Error!.","Presión alta!.","Aceptar.");
        break;
    }
    case 2: {
       // QMessageBox::critical(this,"Error!.","Presión baja!.","Aceptar.");
        break;
    }
    case 3: {
      //  QMessageBox::critical(this,"Error!.","Frecuencia respiratoria alta!.","Aceptar.");
        break;
    }
    case 4: {
       // QMessageBox::critical(this,"Error!.","Frecuencia respiratoria baja!.","Aceptar.");
        break;
    }
    case 5: {
       // QMessageBox::critical(this,"Error!.","O2 alto!.","Aceptar.");
        break;
    }
    case 6: {
       // QMessageBox::critical(this,"Error!.","O2 bajo!.","Aceptar.");
        break;
    }
    case 7: {
       // QMessageBox::critical(this,"Error!.","Volumen bajo!.","Aceptar.");
        break;
    }
    case 8: {
       // QMessageBox::critical(this,"Error!.","Volumen alto!.","Aceptar.");
        break;
    }
    case 9: {
        //ui->label_estado->setText("Estado: Error! fuga o desconexion!");
        //ui->label_estado->setStyleSheet("color: rgb(170, 0, 0)");
       // QMessageBox::critical(this,"Error!.","Fuga o desconexión!.","Aceptar.");
        break;
    }
    }
   /* if(((alarmPressInputO2 == false) && (alarmPressInput == false))){
    digitalWrite(RELE4,LOW);
    }*/
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* ALARMA SONIDO */
void MainWindow::AlarmOut(){
    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(100000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);

    digitalWrite(RELE4,HIGH);
    delayMicroseconds(200000);
    digitalWrite(RELE4,LOW);
    delayMicroseconds(100000);
}

/**************************************************************************************************************************************/
//CONTROL DE VALVULAS
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INALACIÓN */
void MainWindow::valvesMainControl(uint8_t inspirationValves ){
delayMicroseconds(100);
digitalWrite(REL1,(inspirationValves &  0x01));
digitalWrite(REL2,(inspirationValves &  0x02));
digitalWrite(REL4,(inspirationValves &  0x04));
digitalWrite(REL3,(inspirationValves &  0x08));
digitalWrite(REL5,(inspirationValves &  0x10));
digitalWrite(REL6,(inspirationValves &  0x20));
digitalWrite(REL7,(inspirationValves &  0x40));
digitalWrite(REL8,(inspirationValves &  0x80));
delayMicroseconds(5000);
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* EXALACIÓN */
void MainWindow::valvesExControl(uint8_t exalationValves ){
delayMicroseconds(100);
digitalWrite(RELE1,(exalationValves &  0x01));
digitalWrite(RELE2,(exalationValves &  0x02));
digitalWrite(RELE3,(exalationValves &  0x04));
//digitalWrite(RELE4,(exalationValves &  0x08)); //456789
delayMicroseconds(5000);
}


/**************************************************************************************************************************************/
//RANGOS MAXIMOS PARA EL SISTEMA.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SUMA DE MAXIMO LIMITE DE VOLUMEN *//*
void MainWindow::on_pushButton_mor_maxPress_2_clicked()
{
    maxVolLimit+=50;
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE VOLUMEN *//*
void MainWindow::on_pushButton_min_maxPress_2_clicked()
{
    maxVolLimit-=50;
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE PRESION */
void MainWindow::on_pushButton_min_maxPress_clicked()
{
    maxPressLimit -= 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* RESTA DE MAXIMO LIMITE DE PRESION */
void MainWindow::on_pushButton_mor_maxPress_clicked()
{
    maxPressLimit += 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}







/**************************************************************************************************************************************/
//RANGOS DE CONTROL PARA EL SISTEMA.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE PEEP */
void MainWindow::on_pushButton_minPEEP_clicked()
{
    setPEEPr-=1;
    if(setPEEPr<=4) {setPEEPr = 4;}
    ui->label_setPEEP->setText(QString::number(setPEEPr,'f',1));
    evalVel();

}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE PEEP */void MainWindow::on_pushButton_morePEEP_clicked()
{
    setPEEPr+=1;
    if(pressControlActive){
    if(setPEEPr>=(setPIP-3)) {setPEEPr = setPIP-3;}
    }
    if(setPEEPr >= 20){setPEEPr = 20;}
    ui->label_setPEEP->setText(QString::number(setPEEPr,'f',0));
    evalVel();

}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE PIP */
void MainWindow::on_pushButton_morePIP_clicked()
{
    setPIP+=1;
    if(setPIP>=80) {setPIP  = 80;}
    ui->label_press_pip->setNum(setPIP);
    ui->label_pressAlarmLow->setText(QString::number(setPIP-((double(lowPressPorc)/100.0)*setPIP),'f',1));
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE PIP */
void MainWindow::on_pushButton_minPIP_clicked()
{
    setPIP-=1;
    if(setPIP<=6) {setPIP  = 6;}
    if((setPIP-3)<=setPEEPr){
        setPEEPr=setPIP-3;
        ui->label_setPEEP->setText(QString::number(setPEEPr,'f',0));
    }
    ui->label_press_pip->setNum(setPIP);
    ui->label_pressAlarmLow->setText(QString::number(setPIP-((double(lowPressPorc)/100.0)*setPIP),'f',1));
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE VOLUMEN */
void MainWindow::on_pushButton_minVol_clicked()
{
    setVOL-=50;
    if(setVOL<=50) {setVOL = 50;}
    ui->label_press_volsetpoint->setNum(setVOL);
    alarmCurrenCiclosVol = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE VOLUMEN */
void MainWindow::on_pushButton_moreVol_clicked()
{
    setVOL+=50;
    inspirationTimeForVol = ((double(60.0/double(FRv))*1000.0)/(ieRatioRef+1.0))/1000.0;
    //if(setVOL>=2050) {setVOL = 2050;}
    if(setVOL >= ((1200.0*inspirationTimeForVol)-50)) {
        setVOL = 1200.0*inspirationTimeForVol-50;
        setVOL = int((int(setVOL)-25)/50 * 50);
    }
    if(setVOL >= 1200){
        setVOL = 1200;
    }

    ui->label_press_volsetpoint->setNum(setVOL);
    alarmCurrenCiclosVol = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE FR */
void MainWindow::on_pushButton_moreFR_clicked(bool fromIE)
{
    if(fromIE == false){
    FRv=FRv+1;
    }

    if(ui->tabWidget_sel->currentIndex()==1){
        FRvMax=int(60.0/((setVOL/1200.0)*(ieRatioRef+1.0)))-4;
        if(FRv>=FRvMax) {
            FRv = uint8_t(FRvMax);
        }
    }
    if(ui->tabWidget_sel->currentIndex()==0){
        if(FRv>=60) {
            FRv = 60;
        }
    }

    if(FRv >= 60){
        FRv = 60;
    }

    timeFRv = uint32_t(double(60.0/double(FRv))*1000.0);
    ui->label_fr->setNum(int(FRv));
    alarmCurrenCiclosFR = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE FR */
void MainWindow::on_pushButton_minFR_clicked()
{
    FRv=FRv-1;
    if(FRv<=4) {FRv = 4;}
    timeFRv = uint32_t(double(60.0/double(FRv))*1000.0);
    ui->label_fr->setNum(int(FRv));
    alarmCurrenCiclosFR = cicleCounter;
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* INCREMENTO DE SETPOINT DE IE */
void MainWindow::on_pushButton_mor_ie_clicked()
{
    ieRatioRef = ieRatioRef+1;
    if(ieRatioRef>=5) {ieRatioRef = 5;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    on_pushButton_moreFR_clicked(true);
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DECREMENTO DE SETPOINT DE IE */
void MainWindow::on_pushButton_min_ie_clicked()
{
    ieRatioRef = ieRatioRef - 1;
    if(ieRatioRef<=1) {ieRatioRef = 1;}
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    evalVel();
}



/**************************************************************************************************************************************/
//MISC.
/**************************************************************************************************************************************/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* FUNCION DUMMY DE VALIDACIÓN POR TIEMPOS QUITAR EL WHILE PARA PROBAR*/
void MainWindow::validacion(){
    uint8_t valTest = 50;
    for(uint8_t temporal = 0; temporal <= 100; temporal++){
        valvesMainControl(valTest); //Inspiracion
        delayMicroseconds(1000000);
        valvesMainControl(0x00); //Inspiracion

        qDebug()<< "Presión maxima:" << pressureRead();
        valvesExControl(0x01); //Exalación
        delayMicroseconds(6000000);
        valvesExControl(0x00); //Exalación

        valTest = valTest+20;
        qDebug() << valTest;

        if(valTest>=190){
            valTest=30;
        }
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* DETECCIÓN DE FUGAS */
void MainWindow::on_pushButton_pressLeaks_clicked()
{
    valvesExControl(0);
    valvesMainControl(0x10);
    for(int i=0;i<=100;i++){
        pressLeaksData = pressureRead();
        ui->label_pressLeaks->setText(QString::number(pressLeaksData,'f',1));
        ui->progressBar_pressLeak->setValue(i);
        delayMicroseconds(1000);
    }
    ui->label_pressLeaksMax->setText(QString::number(pressLeaksData,'f',1));
    valvesMainControl(0);
    if(pressLeaksData<=5){
        QMessageBox::critical(this,"Error!.","No se detecto presión!.","Aceptar.");
    }
    else{

        for(int i=0;i<=100;i++){
            ui->label_pressLeaks->setText(QString::number(pressureRead(),'f',1));
            ui->progressBar_pressLeak->setValue(100-i);
            delayMicroseconds(1000);
        }
        ui->label_pressLeaksFin->setText(QString::number(pressureRead(),'f',1));
        ui->label_pressLeaksDiff->setText(QString::number(pressLeaksData-pressureRead(),'f',1));


        if(pressLeaksData-pressureRead()>=15){
            QMessageBox::critical(this,"Error!.","Fugas detectadas!.","Aceptar.");
        }
        else {
            QMessageBox::critical(this,"Ok!.","No se detectaron fugas!.","Aceptar.");
        }
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* AJUSTE DE CONFIGURACIÓN */
void MainWindow::on_pushButton_3_clicked()
{
    ui->tabWidget->setCurrentIndex(0);
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* REINICIO DE CONTADOR */
void MainWindow::on_pushButton_resetC_clicked()
{
    cicleCounter = 0;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* IMPRESIÓN DE TIEMPO EN EL DEPURADOR */
void MainWindow::printTimer(QString info){
    timerMillis=millis();
    if(DEBUG_STATUS){   qDebug() << info << timerMillis;}
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* TEST */
/*
void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << "Presion: " << pressureRead();
    qDebug() << "O2: " << o2Read();
    qDebug() << "Flujo: " << flowRead(initI2C);
}*/
/*------------------------------------------------------------------------------------------------------------------------------------*/
/* CAMBIO DE TAB */
void MainWindow::on_pushButton_Conf_clicked()
{
    ui->tabWidget->setCurrentIndex(1);
}


void MainWindow::on_radioButton_name_clicked()
{
    if(ui->radioButton_name->isChecked()){
        ui->lineEdit_textName->setEnabled(true);
    }
    else{
        ui->lineEdit_textName->setEnabled(false);
    }
}

void MainWindow::on_radioButton_date_clicked()
{
    if(ui->radioButton_date->isChecked()){
        ui->lineEdit_textName->setEnabled(false);
    }
    else{
        ui->lineEdit_textName->setEnabled(true);
    }
}


void MainWindow::on_pushButton_10_clicked()
{
    testTimer->start(200);
    ui->pushButton_10->setEnabled(false);
    ui->pushButton_stop->setEnabled(true);

}

void MainWindow::testTimerFunction(){
    testGPIOindex++;
    QString dataDebugText;

    qDebug() << "i:" << testGPIOindex <<"  Presion: " << pressureRead() << "      Flujo: " << flowRead(initI2C) << "  O2: " << o2Read()
    << "P AIR: " << pressSairRead() << "  P O2: " <<  pressSo2Read();
    dataDebugText = "i:" +
            QString::number(testGPIOindex) +
            "   Presion: " +
            QString::number(pressureRead(),'f',2) +
            " cmH2O." +
            "   Flujo: " +
            QString::number(flowRead(initI2C),'f',2) +
            " ml/s." +
            "   O2: " +
            QString::number(o2Read(),'f',2) +
            " %." +
            "   P AIR: " +
            QString::number(pressSairRead()) +
            "   P O2: " +
            QString::number(pressSo2Read());
    ui->label_dataDebug->setText(dataDebugText);
}


void MainWindow::on_pushButton_stop_clicked()
{
    testTimer->stop();
    ui->pushButton_stop->setEnabled(false);
    ui->pushButton_10->setEnabled(true);


}



void MainWindow::on_pushButton_2_clicked()
{
    uint8_t inalacion=0;
    uint8_t exalacion=0;
    if(ui->checkBox->isChecked()){inalacion+=0x01;}
    if(ui->checkBox_2->isChecked()){inalacion+=0x02;}
    if(ui->checkBox_3->isChecked()){inalacion+=0x04;}
    if(ui->checkBox_4->isChecked()){inalacion+=0x08;}
    if(ui->checkBox_5->isChecked()){inalacion+=0x10;}
    if(ui->checkBox_6->isChecked()){inalacion+=0x20;}
    if(ui->checkBox_7->isChecked()){inalacion+=0x40;}
    if(ui->checkBox_8->isChecked()){inalacion+=0x80;}

    if(ui->checkBox_E1->isChecked()){exalacion+=0x01;}
    if(ui->checkBox_E2->isChecked()){exalacion+=0x02;}
    if(ui->checkBox_E3->isChecked()){exalacion+=0x04;}
    if(ui->checkBox_E4->isChecked()){exalacion+=0x08;}


    qDebug() << "Inalacion: " << inalacion << " || Exalación: " << exalacion;

    valvesMainControl(inalacion);
    valvesExControl(exalacion);
}


void MainWindow::on_horizontalSlider_sensibilidad_sliderMoved(int position)
{
    ui->label_sensibilidad->setText(QString::number(position));
}

void MainWindow::evalVel(){
        if(ui->tabWidget_sel->currentIndex()==1){
            inspirationTime = (double(60.0/double(FRv))*1000.0)/(ieRatioRef+1.0);
            valvesValueControl=int(setVOL/((inspirationTime/1000.0)*5.0));
        }
        if(ui->tabWidget_sel->currentIndex()==0 || ((ui->tabWidget_sel->currentIndex()==2) && (pressControlActive == true))){

            if(ui->tabWidget_sel->currentIndex()==0){
                if(int(ieRatioRef) >= 3){
                    if(FRv >=9 && FRv < 12){
                        if(setPIP <8)                   {valvesValueControl = 8; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 15; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 23; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 32; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 45; }
                        if(setPIP>=20)                  {valvesValueControl = 53; }
                    }
                    if(FRv >=12 && FRv < 15){
                        if(setPIP <8)                   {valvesValueControl = 10; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 16; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 28; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 44; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 55; }
                        if(setPIP>=20)                  {valvesValueControl = 62; }
                    }
                    if(FRv >=15 && FRv < 18){
                        if(setPIP <8)                   {valvesValueControl = 15; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 23; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 36; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 53; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 63; }
                        if(setPIP>=20)                  {valvesValueControl = 75; }
                    }
                    if(FRv >=18 && FRv < 21){
                        if(setPIP <8)                   {valvesValueControl = 15; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 29; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 39; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 59; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 76; }
                        if(setPIP>=20)                  {valvesValueControl = 86; }
                    }
                }
                if(int(ieRatioRef) == 2){
                    if(FRv >=9 && FRv < 12){
                        if(setPIP <8)                   {valvesValueControl = 10; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 12; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 18; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 24; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 31; }
                        if(setPIP>=20)                  {valvesValueControl = 38; }
                    }
                        if(FRv >=12 && FRv < 15){
                        if(setPIP <8)                   {valvesValueControl = 11; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 15; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 28; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 44; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 53; }
                        if(setPIP>=20)                  {valvesValueControl = 62; }
                    }
                    if(FRv >=15 && FRv < 18){
                        if(setPIP <8)                   {valvesValueControl = 12; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 22; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 31; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 47; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 63; }
                        if(setPIP>=20)                  {valvesValueControl = 71; }
                    }
                    if(FRv >=18 && FRv < 21){
                        if(setPIP <8)                   {valvesValueControl = 14; }
                        if(setPIP>=8  && setPIP <11)    {valvesValueControl = 23; }
                        if(setPIP>=11 && setPIP <14)    {valvesValueControl = 38; }
                        if(setPIP>=14 && setPIP <17)    {valvesValueControl = 54; }
                        if(setPIP>=17 && setPIP <20)    {valvesValueControl = 70; }
                        if(setPIP>=20)                  {valvesValueControl = 78; }
                    }

                }
                if(setPIP>=20){
                    valvesValueControl+=(setPIP-20)*2;
                }
               //valvesValueControl = valvesValueControl*0.9;
            }

        if((setPEEP>=5) && (pressControlActive == true)){

            if(setPEEP>=10){
                valvesValueControl-=(setPEEP-7.0)*1.1;
             }
            else{
                valvesValueControl-=(setPEEP-4.0)*1.6;
            }



             if(valvesValueControl<=1){
                 valvesValueControl = 1;
             }
             qDebug() << "Nuevo valor de PEEP COMP: " << (setPEEP-5.0)*3.0;

        }
        }

        qDebug() << "Nuevo valor de velocidad: " << valvesValueControl;
}

void MainWindow::on_pushButton_11_clicked()
{
    inspirationDetected = true;
}

void MainWindow::on_pushButton_alarmTest_clicked()
{
    activateAlarm(1);
}


QString MainWindow::initSystem(){
    bool testInitSystem = true;
    QString initSystemString = "Analisis de sistema:\n";

    qDebug() << "Inicializando sistema.";

    initI2C = wiringPiI2CSetup(0x49); // Inicializar el sensor de flujo Honeywell
    qDebug() << "I2C init: " << initI2C;
    delayMicroseconds(100000);

    wiringPiI2CWrite(initI2C,0x01);
    delayMicroseconds(100000);
    int serial1 = wiringPiI2CReadReg16(initI2C,0x00);
    int serial2 = wiringPiI2CReadReg16(initI2C,0x00);

    qDebug() << "I2C serial number : " << serial1 << " " << serial2; //Verificando serial del sensor.
    wiringPiI2CWrite(initI2C,0x03);
    delayMicroseconds(100000);
    serial1 = wiringPiI2CReadReg16(initI2C,0x00);
    dataH = uint8_t(serial1);
    dataL = uint8_t(serial1>>8);
    dataFull = uint16_t(dataH<<8)+dataL;

    if(dataFull==52389){
        qDebug() << "I2C CheckSum del sensor: " << dataFull;
        testInitSystem = true;
        initSystemString+="I2C Flujo: OK!\n";
    }
    else{
        qDebug() << "Error no  se detecta sensor de flujo!";
        testInitSystem = false;
        initSystemString+="I2C Flujo: NOK!\n";
    }

    delayMicroseconds(100000);

    //----------------------------------------------------------------

    if(wiringPiSetupGpio()){
        qDebug() << "Error en perifericos.";
        testInitSystem = false;
        initSystemString+="Perifericos: NOK!\n";
    }
    else{
        qDebug() << "Inicializando perifericos.";
        testInitSystem = true;
        initSystemString+="Perifericos: OK!\n";
    }

    if(initFile()){
        qDebug() << "Documento inicializado.";
        initSystemString+="Memoria: OK!\n";
        testInitSystem = true;
    }
    else{
        qDebug() << "Error en memoria.";
        initSystemString+="Memoria: NOK!\n";
        testInitSystem = false;
    }

    pinMode(REL1,OUTPUT);
    pinMode(REL2,OUTPUT);
    pinMode(REL3,OUTPUT);
    pinMode(REL4,OUTPUT);
    pinMode(REL5,OUTPUT);
    pinMode(REL6,OUTPUT);
    pinMode(REL7,OUTPUT);
    pinMode(REL8,OUTPUT);

    pinMode(RELE1,OUTPUT);
    pinMode(RELE2,OUTPUT);
    pinMode(RELE3,OUTPUT);
    pinMode(RELE4,OUTPUT);
    pinMode(AIR_PRESS_ALARM,INPUT);
    pinMode(O2_PRESS_ALARM,INPUT);
    pinMode(ValveExp,OUTPUT);

    // Inicializando I2C / ADC 1115
    if(ads1115Setup(AD_BASE,0x48) < 0){
        qDebug() << "Error en el I2C con el ADC";
        testInitSystem = false;
        initSystemString+="I2C ADC: NOK!\n";
    }
    else{
        qDebug() << "ADC Inicializado.";
        testInitSystem = true;
        initSystemString+="I2C ADC: OK!\n";
    }

    ui->pushButton_stop->setEnabled(false); //Poner botón de parar prueba en falso

    digitalWrite(AD_BASE,2); // COnfigurando lla referencia de 2V.

    timerStatusFlag=false;

    sensorTimer->setTimerType(Qt::PreciseTimer);
    plotTimer->setTimerType(Qt::PreciseTimer);
    controlTimer->setTimerType(Qt::PreciseTimer);

    QObject::connect(sensorTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::sensorTimerFunction));
    QObject::connect(plotTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::plotTimerFunction));
    QObject::connect(controlTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::controlTimerFunction));
    QObject::connect(testTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::testTimerFunction));
    QObject::connect(activeTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::activeTimerFunction));
    QObject::connect(stopTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::stopTimerFunction));

    stopTimer->start(250);

    activeTimer->start(1000);

    pressData.insert(0,251,0.0);
    volData.insert(0,251,0.0);
    flowData.insert(0,251,0.0);
    PEEPData.insert(0,251,0.0);

    increaseVolTemp=0;
    volTemp=0;

    ui->label_press_pip->setNum(setPIP);
    ui->label_setPEEP->setNum(int(setPEEPr));
    ui->label_fr->setNum(int(FRv));

    pressurePIP=false;
    pressure0=false;
    pressureMAX = false;

    ieRatioRef = 2;

    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
    ui->label_errorFlow->setText(QString::number(flowError,'f',0));
    ui->label_frError->setText(QString::number(frError,'f',0));
    ui->label_o2setpoint_error->setText(QString::number(fio2Error,'f',0));
    ui->label_o2setpoint->setText(QString::number(fioSetPoint,'f',0));
    ui->label_loPressPorcent->setText(QString::number(lowPressPorc));
    ui->label_pressAlarmLow->setText(QString::number(setPIP-((double(lowPressPorc)/100.0)*setPIP),'f',1));

    getDateText();
    plotSetup(ui->customPlot);

    inspirationDetected2 = true;

    for(int i=0;i<=20;i++){
        readedO2 += o2Read();
    }

    readedO2 = readedO2/20;
    fioSetPoint = readedO2;
    ui->label_o2setpoint->setText(QString::number(readedO2,'f',0));
    ui->label_o2->setText(QString::number(readedO2,'f',1));
    ui->tabWidget->setCurrentIndex(0);
    ui->tabWidget_sel->setCurrentIndex(0);

    QFile file("/home/pi/TimeActive.txt");
    if(file.open(QFile::ReadOnly | QFile::Text)){
    }

    QTextStream in(&file);
    QString timeActiveRead = in.readAll();
    file.flush();
    file.close();

    timeActive = QDateTime::fromString(timeActiveRead,"d HH:mm:ss");
    ui->label_activeTime->setText(timeActive.toString("d HH:mm:ss"));
    timeActiveVent = QDateTime::fromString("1 00:00:00","d HH:mm:ss");
    ui->label_activeTimeVent->setText(timeActiveVent.toString("d HH:mm:ss"));

    valvesMainControl(0);
    valvesExControl(0);
    AlarmOut();

    double testPress = pressureRead();
    double testO2 = o2Read();
    qDebug() << "O2: " << testO2;

    if((testPress>=-1) && (testPress<=50)){
        testInitSystem = true;
        initSystemString+="Data de presión: OK!\n";
    }
    else{
        testInitSystem = false;
        initSystemString+="Data de presión: NOK!\n";
    }

    if((testO2>=0) && (testO2<=100)){
        testInitSystem = true;
        initSystemString+="Data de O2: OK!\n";
    }
    else{
        testInitSystem = false;
        initSystemString+="Data de O2: NOK!\n";
    }

    if(pressSo2Read()<= 13300){
        testInitSystem = false;
        initSystemString+="Presión de O2: NOK!\n";
    }
    else{
        initSystemString+="Presión de O2: OK!\n";
    }

    if(pressSairRead()<= 12600){
        testInitSystem = false;
        initSystemString+="Presión de Aire: NOK!\n";
    }
    else{
        initSystemString+="Presión de Aire: OK!\n";
    }

    initSystemString+="\n------------------------------------------------------\n\n\n\n\n";

    ui->tabWidget->setTabEnabled(3, false);
    ui->tabWidget->setTabEnabled(4, false);
    ui->tabWidget->setTabEnabled(5, false);

    ui->label_estado->setStyleSheet("color: rgb(0, 0, 0)");
    //ui->label_ajuste->setStyleSheet("color: green; font-weight: 400");
    ui->label_estado->setText("Estado: ");

    readedO2 = fioSetPoint;
    if(testInitSystem){
        qDebug() << "Todo OK";
    }
    else{
        qDebug() << "No OK";
    }

    return initSystemString;
}

void MainWindow::on_pushButton_alarmTest_3_clicked()
{
    digitalWrite(RELE4,LOW);
    silence = true;
    frCounter = 0;
    fio2Counter = 0;
    vmCounter = 0;
    alarmTimeStop = millis();
    alarmOn = false;
    /* Registro de eventos */ // "0003"
    eventLogText=ui->label_eventError->text();
    eventLogText+="\n";
    eventLogText+=alarmText;
    eventLogText+=" ";
    eventLogText+=dateTimeSys.toString();
    ui->label_eventError->setText(eventLogText);

    alarmText = "";
    ui->label_estado->setStyleSheet("color: rgb(0, 0, 0)");
    //ui->label_ajuste->setStyleSheet("font-weight: 400; color: rgb(0, 85, 0)");
    ui->label_estado->setText("Estado: ");
    //ui->label_ajuste->setStyleSheet("font-weight: 400; color: rgb(0, 85, 0)");

    slopeFlat = false;
    timeFlatSlope = millis();
}





void MainWindow::on_tabWidget_sel_currentChanged(int index)
{
    evalVel();
    if(index==0){
        pressControlActive = true;
        volControlActive = false;
        alarmCurrenCiclosVol = cicleCounter;
        if((setPIP-3)<=setPEEPr){
            setPEEPr=setPIP-3;
            ui->label_setPEEP->setText(QString::number(setPEEPr,'f',0));
        }
    }
    else if (index==1){
        pressControlActive = false;
        volControlActive = true;
    }
    qDebug() << "Press: " << pressControlActive << " Vol: " << volControlActive;
}


void MainWindow::on_pushButton_alarmTest_2_clicked()
{
    testError=!testError;
}


void MainWindow::on_horizontalSlider_pressSlope_sliderMoved(int position)
{
    slopePressureAdj = ((double(position)-500.0)/500)+1;
    ui->label_slopePress->setText(QString::number(slopePressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_pressOffset_sliderMoved(int position)
{
    offsetPressureAdj = (double(position)/1000.0)*5.0;
    ui->label_offsetPress->setText(QString::number(offsetPressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_flowSlope_sliderMoved(int position)
{
    slopePressureAdj = (double(position)-500.0)/500.0+1;
    ui->label_slopeFlow->setText(QString::number(slopePressureAdj,'g',3));
}

void MainWindow::on_horizontalSlider_flowOffset_sliderMoved(int position)
{
    offsetFlowAdj = (double(position)/1000.0)*5.0;
    ui->label_offsetFlow->setText(QString::number(offsetFlowAdj,'g',3));
}

void MainWindow::on_pushButton_min_maxVol_clicked()
{
    flowError-=1;
    if(flowError <= 0){
        flowError = 0;
    }
    ui->label_errorFlow->setText(QString::number(flowError,'f',0));
}

void MainWindow::on_pushButton_mor_maxVol_clicked()
{
    flowError+=1;
    ui->label_errorFlow->setText(QString::number(flowError,'f',0));
}

void MainWindow::on_pushButton_min_fio2set_clicked()
{
    fioSetPoint-=1;
    ui->label_o2setpoint->setText(QString::number(fioSetPoint,'f',0));
}

void MainWindow::on_pushButton_mor_fio2set_clicked()
{
    fioSetPoint+=1;
    ui->label_o2setpoint->setText(QString::number(fioSetPoint,'f',0));
}

void MainWindow::on_pushButton_min_fio2setError_clicked()
{
    fio2Error-=1;
    ui->label_o2setpoint_error->setText(QString::number(fio2Error,'f',0));
}

void MainWindow::on_pushButton_mor_fio2setError_clicked()
{
    fio2Error+=1;
    ui->label_o2setpoint_error->setText(QString::number(fio2Error,'f',0));
}

void MainWindow::on_pushButton_min_frError_clicked()
{
    frError-=1;
    if(frError <= 0){
        frError = 0;
    }
    ui->label_frError->setText(QString::number(frError,'f',0));
}

void MainWindow::on_pushButton_mor_frError_clicked()
{
    frError+=1;
    ui->label_frError->setText(QString::number(frError,'f',0));
}

void MainWindow::initVariables(void){
    offsetPEEP = -2;
}

void MainWindow::stopTimerFunction(void){
    readedO2 = o2Read();

    O2Prom[0] = O2Prom[1]; O2Prom[1] = O2Prom[2]; O2Prom[2] = O2Prom[3]; O2Prom[3] = O2Prom[4]; O2Prom[4] = O2Prom[5]; O2Prom[5] = readedO2;
    double readedO2Prom = (O2Prom[0] + O2Prom[1] + O2Prom[2] + O2Prom[3] + O2Prom[4] + O2Prom[5])/6.0;
    readedO2 = readedO2Prom;

    if(readedO2>=105){
        readedO2 = 105;
    }

    ui->label_o2->setText(QString::number(readedO2,'f',0));
    ui->label_currentFio2->setText(QString::number(readedO2,'f',1));


    if(ui->tabWidget_sel->currentIndex()==2){
        ui->pushButton_start->setEnabled(false);
    }
    else{
        ui->pushButton_start->setEnabled(true);
    }
    //qDebug() << "Stop O2: " << readedO2;
}

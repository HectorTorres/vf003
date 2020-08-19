#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "wiringPi.h"
#include "softPwm.h"
#include "ads1115.h"

#include "passwindow.h"
#include "homewidow.h"

#include <QTimer>
#include <QDateTime>
#include <QFile>
#include <QTextStream>

#include "qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    int dutyCicle=0;
    void plotTimerFunction();
    void sensorTimerFunction(void);
    void controlTimerFunction(void);
    void testTimerFunction(void);
    void activeTimerFunction(void);
    void validacionFunction(void);
    void plotSetup(QCustomPlot *customPlot);
    double pressureRead(void);
    double flowRead(int I2Cfn);
    double volRead(double flowIn);
    double o2Read(void);
    double pressSo2Read(void);
    double pressSairRead(void);
    void plotData(QCustomPlot *customPlot);
    void printTimer(QString info);
    bool initFile(void);
    void writeFile(QString textToFile);
    void getDateText();
    void evalVel();
    void activateAlarm(uint16_t number);
    void AlarmOut();
    void valvesMainControl(uint8_t inspirationValves);
    void valvesExControl(uint8_t exalationValves );
    void validacion(void);
    void initVariables(void);
    void stopTimerFunction(void);

    QString initSystem(void);

    unsigned int timerMillis;
    int endLineUpStatus, endLineDownStatus;
    bool pressurePIP, pressure0, pressureMAX;
    bool volMAX;
    QTimer *plotTimer = new QTimer(this);
    QTimer *sensorTimer = new QTimer(this);
    QTimer *controlTimer = new QTimer(this);
    QTimer *activeTimer = new QTimer(this);
    QTimer *testTimer = new QTimer(this);
    QTimer *stopTimer = new QTimer(this);

    QDateTime timeActive;
    QDateTime timeActiveVent;
    uint8_t timeActiveToSave;

    QString time_format = "yyyy-MM-dd HH:mm:ss";
    QDateTime dateTimeSys;

    bool timerStatusFlag;
    uint16_t vel=200;
    uint8_t ratio;
    bool checkLineEnd = true;
    uint32_t cicleCounter=0;
    QVector<double> pressData;
    QVector<double> volData;
    QVector<double> flowData;
    QVector<double> o2Data;
    QVector<double> x;
    QVector<double> PEEPData;

    double readedPress, readedPressTemp;
    double readedPressTempD=6;
    double readedFlow;
    double readedFlowTempD = 5;
    double readedVol=120;
    double readedVolTempD;
    double readedO2, readedO2Temp;
    int pressLowAlarm = 0;
    int lowPressPorc = 20;

    double adjustPEEP = 0;

    double volTemp, increaseVolTemp;
    bool pauseVol=false;
    int indexPress=0;
    double setPIP = 12;
    double setVOL = 250;
    double minPEEP = 5;
    double setPEEP = 4;
    double setPEEPr = 4;

//pushButton_min_fio2setError
    double offsetPressure = 0;
    double offsetFlow = 0;
    double offsetPressureTemp = 0;
    double offsetFlowTemp = 0;

    bool noPress=false;
    unsigned int noPressTime;
    bool assistPressDetect = false;

    QVector<double> pressSlope{0,0,0,0,0} ;
    QVector<double> pressProm{0,0,0,0,0,0,0,0,0,0};
    QVector<double> pressProm2{0,0,0};
    QVector<double> flowProm{0,0,0,0,0,0,0,0,0,0};
    QVector<double> IeProm{0,0,0,0,0,0};
    QVector<double> O2Prom{0,0,0,0,0,0};

    unsigned int timePeriodA, timePeriodB, periodMotor;
    unsigned int fR=13;
    unsigned int periodfRmilis=0;

    unsigned int ieRatioUp, ieRatioUpPeriod, ieRatioDown, ieRatioDownPeriod;
    unsigned int motorPauseDown=0;
    bool ieFlag=false;
    bool ieFlag2 = false;
    bool ieFlag3 = false;
    bool ieNewTemporal = false;

    bool inspirationDetected = false;
    bool inspirationDetectedevent = true;

    double ieRatioVal, ieRatioRef, ieRatio, ieRatioFracc, FRvMax;

    int ieRatioVel = 0;
    int diffTemp = 0;

    unsigned int timeFromInit =0 ;

    unsigned int timeLowPress = 0;
    bool timeLowPressStatus = false;

    int w=0;


    uint16_t distance;
    uint32_t volTimeUp, volTimeDown;
    uint32_t slopeFilterTime;
    uint32_t inspirationInitTime;
    double volTime;
    uint16_t volCounter = 0;
    uint32_t volTimeMaxUp;
    uint8_t errorFrCounter=0;

    QString nameFile = "tempral.csv";
    QString dirFile = "/home/pi/Desktop/dataHMI/";

    QFile file;
    QTextStream dataToFile;

    bool valveStatus = false;

    bool inspirationDetected2 = false;
    bool VL52L0Xinit = false;
    uint16_t distancemm;

    bool volSetPoint = false;
    bool testError = false;
    int i=0;

    //----------------------------------New Sistem-------------------
    double currentMaxPress = 0;
    double maxReadedPress = 0;

    QString estadoDesconexion = "Conectado";

    bool readedPressBottom = false;
    bool readedPressTop = false;

    int valvesValueControl = 40; //Antes como uint32_t

    uint32_t espontaneoPeriod;

    uint8_t FRv=20;
    uint32_t timeFRv = uint32_t(double(60.0/double(FRv))*1000);

    uint32_t timeFRvTemporal = 0;
    uint32_t timeFRvReal = 0;
    uint32_t timeUpIe = 0;
    uint32_t timeDownIe = 0;
    uint32_t timerSlopeIncrease = 0;
    uint32_t timeMasterControl = 0;
    uint32_t timeFromStart = 0;
    uint32_t timeFlatSlope = 0;
    uint32_t timeInPeak = 0;
    uint32_t timeInPeakRead = 0;

    uint8_t inspirationValvesV = 16;
    uint8_t exalationValvesV = 0x3F;
    double IeRatio = 0;
    double IeRatioSetPoint = 0.33;
    double timeFRvRealf = 0;

    double periodTime, periodTimeRead, inspirationTime, inspirationTimeForVol;

    bool inspirationTimeTop = false;
    bool periodTimeTop = false;

    bool flowDataReadStatus=true;

    bool slopeFlat = false;
    uint32_t timeSlope;

    bool vavleChange=false;
    bool slopeIncrease = false;


    double offsetPressureTempInit, offsetFlowTempInit;
    double offsetPressureBits, offsetFlowBits;

    double offsetPressureAdj = 0;
    double offsetFlowAdj = 0;
    double slopePressureAdj = 1;
    double slopeFlowAdj = 1;

    double pressLeaksData;

    double volError, pressError;

    bool alarmPressInput = false;
    bool alarmPressInputO2 = false;

    bool alarmControl = false;

    bool PEEPOnControl = false;
    uint8_t indexPEEP = 0;
    double PEEPaverage = 0;

    /*-----------------------------*/
    //For new flow sensor.
    uint8_t  dataH;
    uint8_t  dataL;
    uint16_t dataFull;
    int initI2C;

    /*-----------------------------*/
    //New control system
    double Kp = 3.00;
    bool pressControlActive = true;
    bool volControlActive = false;
    double offsetPEEP = -2;

    /*-----------------*/
    //Password window

    /*-----------------------------*/
    //Alarmas
    double flowError = 80;
    double frError = 20;
    double frCurrent = 0;
    double fio2Error = 50;
    double fioSetPoint = 20;
    unsigned int maxPressLimit = 80;
    unsigned int maxVolLimit = 950;
    QString alarmText;
    QString eventLogText;
    bool alarmOn = false;
    double volVTI;

    double maxFlow;
    uint32_t alarmCurrenCiclosVol = 0;
    uint32_t alarmCurrenCiclosFR = 0;
    bool noConection = false;
    bool ventMainAlarmSystem=false;
    uint32_t MainAlarmSystemTimer = 0;

    bool flagAlarmPresMax = false;
    bool flagAlarmPressLos = false;
    bool flagAlarmVolMinMax = false;
    bool flagAlarmVolMinMin = false;
    bool flagAlarmFiO2Max = false;
    bool flagAlarmFiO2Min = false;
    bool flagAlarmFRMax = false;
    bool flagAlarmFRMin = false;
    bool flagAlarmDesc = false;
    bool flagAlarmAirPres = false;
    bool flagAlarmO2Pres = false;
    bool silence = false;

    uint32_t TimeflagAlarmPresMax = 0;
    uint32_t TimeflagAlarmPressLos = 0;
    uint32_t TimeflagAlarmVolMinMax = 0;
    uint32_t TimeflagAlarmVolMinMin = 0;
    uint32_t TimeflagAlarmFiO2Max = 0;
    uint32_t TimeflagAlarmFiO2Min = 0;
    uint32_t TimeflagAlarmFRMax = 0;
    uint32_t TimeflagAlarmFRMin = 0;
    uint32_t TimeflagAlarmDesc = 0;
    uint32_t TimeflagAlarmAirPres = 0;
    uint32_t TimeflagAlarmO2Pres = 0;
    uint32_t Timesilence = 0;







    uint32_t alarmTimeStop = 0;
    uint32_t alarmTimeRunn = 0;
    int frCounter = 0;
    int vmCounter = 0;
    int fio2Counter = 0;


    /*--------------*/ //MISC
    uint32_t testGPIOindex = 0;

    public slots:
    void changeModeService(void);


    private slots:
    void on_pushButton_3_clicked();

    void on_pushButton_resetC_clicked();

    void on_pushButton_start_clicked();
    
    void on_pushButton_morePIP_clicked();

    void on_pushButton_minPIP_clicked();

    void on_pushButton_moreFR_clicked(bool fromIE);

    void on_pushButton_minFR_clicked();

    void on_pushButton_mor_ie_clicked();

    void on_pushButton_min_ie_clicked();

    void on_pushButton_min_maxPress_clicked();

    void on_pushButton_mor_maxPress_clicked();

    void on_pushButton_Conf_clicked();

    void on_radioButton_name_clicked();

    void on_radioButton_date_clicked();

    void on_pushButton_minVol_clicked();

    void on_pushButton_moreVol_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_2_clicked();

    void on_horizontalSlider_sensibilidad_sliderMoved(int position);

    void on_pushButton_11_clicked();

    void on_pushButton_alarmTest_clicked();

    void on_tabWidget_sel_currentChanged(int index);

    void on_pushButton_minPEEP_clicked();

    void on_pushButton_morePEEP_clicked();

    void on_pushButton_alarmTest_2_clicked();

    void on_pushButton_pressLeaks_clicked();

    //void on_pushButton_mor_maxPress_2_clicked();

    //void on_pushButton_min_maxPress_2_clicked();

    void on_horizontalSlider_pressSlope_sliderMoved(int position);

    void on_horizontalSlider_pressOffset_sliderMoved(int position);

    void on_horizontalSlider_flowSlope_sliderMoved(int position);

    void on_horizontalSlider_flowOffset_sliderMoved(int position);

    void on_pushButton_min_maxVol_clicked();

    void on_pushButton_mor_maxVol_clicked();

    void on_pushButton_alarmTest_3_clicked();

    void on_pushButton_service_clicked();

    void on_pushButton_min_fio2set_clicked();

    void on_pushButton_mor_fio2set_clicked();

    void on_pushButton_min_fio2setError_clicked();

    void on_pushButton_mor_fio2setError_clicked();

    void on_pushButton_min_frError_clicked();

    void on_pushButton_mor_frError_clicked();

    void on_pushButton_testGPIO_clicked();

    void on_radioButton_esp_clicked();

    void on_radioButton_assit_clicked();

    void on_radioButton_manda_clicked();

    void on_pushButton_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_min_lowPress_clicked();

    void on_pushButton_mor_lowPress_clicked();

    void on_pushButton_moreFR_clicked();

private:
    Ui::MainWindow *ui;
    PassWindow* passWindowCon;
};

#endif // MAINWINDOW_H

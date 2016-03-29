#ifndef SEVER_H
#define SEVER_H

#include "stdafx.h"
#include "DataBuffer.h"

typedef struct SeverBuffer
{
	INT nType;
	CHAR bData[MAX_FRAME_LENGTH];
};

typedef struct ProofreadMessage
{
	CameraSpacePoint pSensor;
	DOUBLE fLowDepth, fHighDepth;
};


typedef struct ThreadParam
{
	SOCKET *sSever, *sClient; //socket var of thread
	SOCKADDR_IN *addrSever, addrClient; //address of sever and client
	INT nClientIndex, nCurrentDelayStamp; //index of client, index of current timming message
	BOOL bStopThread, *mSensorPos; //if thread is ended & CurrentSensorPos is changeable
	HRC *hrc; 
	LLONG *nDelay; //delay of the client
	DataBuffer *bSever; //point to the Trilateration buffer
	CameraSpacePoint *pCurrentSensor; //position of sensor
	INT *nCalibration,*nConnected; //state of client 
	DOUBLE fDiff; //difference of depth in calibration
};


class KinectSever
{

public:
	KinectSever(INT nClient, INT *NPortList, HRC *hrc, DataBuffer *bSever);
	~KinectSever();


	void InitSever();
	//ע��˿��Լ�socket,���������߳�

	LLONG GetDelay(INT nClientIndex);
	//��ȡ�ͻ�����ʱ


	static INT DataSend(SOCKET *sClient, void *data, INT nLength, INT nType);
	//���ݷ��ͺ���

	CameraSpacePoint *GetSenorPos();
	//get state of Calibration



	INT ClientsConnected();
	//return true if all the client is currently connected

	INT Calibration();
	//return true if all clients has sent the current position

	void CalibrationProcess(CameraSpacePoint *pSensor, DOUBLE *fDiff);
	//do the coordinate transformation

	void EndCalibration();

private:
	HRC *Clock;

	DataBuffer *bSever;

	INT nClient;
	//�ͻ�������
	INT nPortList[MAX_SENSOR];
	//�˿��б�
	LLONG nDelay[MAX_SENSOR];
	//�ͻ�����ʱ
	std::mutex mDelay[MAX_SENSOR];
	//�ӳ�������
	INT nConnected;
	//state of the clients

	INT nCalibration;
	//number of Calibration clients .

 
	CameraSpacePoint pCurrentSensor[MAX_SENSOR];
	DOUBLE fDiff[MAX_SENSOR];
	
	BOOL mSensorPos;
	//lock if the position of the all sensors have been calibrated

	WSADATA wsaData;
	ThreadParam param[MAX_SENSOR];
	std::thread threadTime[MAX_SENSOR], threadRecv[MAX_SENSOR];

	SOCKET sSever[MAX_SENSOR];
	SOCKET sClient[MAX_SENSOR];
	SOCKADDR_IN addrSever[MAX_SENSOR];
	//�������ͻ���socket

	void InitAddr(SOCKADDR_IN *addrInput, ULONG addr, ADDRESS_FAMILY family, USHORT port);


	static void ThreadWaitClient(ThreadParam *paramr);
	//�����ͻ����߳�

	static void ThreadTiming(ThreadParam *param);
	//Уʱ֡�����߳�

	static void ThreadDataReceive(ThreadParam *param);
	//���ݽ����߳�

	static void TimingMessageProc(ThreadParam *param, CHAR *buffer);
	//deal with timing message

	static void TrilaterationMessageProc(ThreadParam *param, CHAR *buffer);
	//deal with Trilateration message

	static void CalibrationMessageProc(ThreadParam *param, CHAR *buffer);
	//deal with Calibrationing message

	static void ThreadProc(void *pvoid);
	//virtual proc for thread

};


#endif
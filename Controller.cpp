#include "Controller.h"



Controller::Controller(LPCWSTR &sFileName, HWND *hwnd, HINSTANCE *hInstance)
{
	//HANDLE fConfig = CreateFile(sFileName, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	//CHAR bFileBuffer[50];
	//DWORD nRead;
	nClient = GetPrivateProfileInt(TEXT("Controller"), TEXT("nClient"), 1, sFileName);
	//this->nClient = nClient;
	
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		CHAR cKey[30];
		WCHAR cwKey[30];
		sprintf_s(cKey, "Port%d", i1,30);
		MultiByteToWideChar(CP_ACP, 0, cKey, strlen(cKey) + 1, cwKey, sizeof(cwKey) / sizeof(cwKey[0]));
		nPortList[i1] = GetPrivateProfileInt(TEXT("Controller"), cwKey, 1992, sFileName);
	}

	bKinect = new DataBuffer(nClient);

	sKinect = new KinectSever(nClient, nPortList, &Clock, bKinect);


	this-> hwnd = hwnd;

	this-> hInstance = hInstance;
	
	nCalibrationInterface = 0;

	RECT rVision;
	SetRect(&rVision,
		310,
		50,
		310 + DISPLAY_WIDTH,
		50 + DISPLAY_HEIGHT);
	vision = new Vision(rVision,
		hwnd,
		2);
}

Controller::~Controller()
{

}


void Controller::StartController()
{
	sKinect->InitSever();

	param.hwnd = hwnd;
	param.hInstance = hInstance;
	param.hwndCalibrationButton = &hwndCalibrationButton;
	param.hwndMarkDistance = &hwndMarkDistance;
	param.buffer = bKinect;
	param.hrc = &Clock;
	param.sever = sKinect;
	param.vision = vision;

	param.nClient = nClient;
	param.pSensor = pSensor;
	param.nCalibrationInterface = &nCalibrationInterface;
	param.nCalibrationState = &nCalibrationState;
	param.fAngle = fAngle;


	std::thread threadRefresh = std::thread(ThreadRefresh,&param);
	threadRefresh.detach();
}

void Controller::ThreadRefresh(ThreadParamController *param)
{
	TrilaterationNode *pTemp[MAX_SENSOR];
	CameraSpacePoint pBodyCurrent[BODY_LENGTH];
	BOOL bCalc = FALSE;
	INT nConnected;
	TIMETICK nThreadTimeStamp;

			
	HANDLE fLogController;
	fLogController = CreateFile(TEXT("log_controller.txt"), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);


	//make log file


	while(TRUE)
	{
		param->hrc->TimeStampStart(&nThreadTimeStamp);
		nConnected = param->sever->ClientsConnected();
		if (nConnected == (1 << param->nClient) - 1)
		{
			if (*param->nCalibrationInterface == 0)
				*param->nCalibrationInterface = 1;
		}
		if (*param->nCalibrationState == 2)
		{
			if (param->buffer->RefreshData(param->buffer))
			{
				for (INT i1 = 0; i1 < param->nClient; i1++)
				{
					//get the joint data from buffer
					pTemp[i1] = param->buffer->PickData(i1, param->buffer);
				}

				//made log start

				CHAR bFile[256];
				DWORD nWrite;
				LLONG ntTime = param->hrc->HighResolutionTime();
				sprintf_s(bFile, 256, "Time: %d\n", ntTime);
				WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);
				for (INT i1 = 0; i1 < param->nClient; i1++)
				{
					sprintf_s(bFile, 256, "Sensor#%d, [%.1f %.1f %.1f]\n",
						i1,
						param->pSensor[i1].X,
						param->pSensor[i1].Y,
						param->pSensor[i1].Z);
					WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);
				}


				//made log end





				for (INT i1 = 0; i1 < BODY_LENGTH; i1++)
				{
					CameraSpacePoint pStored[MAX_SENSOR];
					INT nShade[MAX_SENSOR];
					DOUBLE fSpeed[MAX_SENSOR];




					for (INT i2 = 0; i2 < param->nClient; i2++)
					{
						CameraSpacePoint pTempTrans;
						pTempTrans.X = pTemp[i2]->data.pBody[i1].Y;
						pTempTrans.Y = pTemp[i2]->data.pBody[i1].X;
						pTempTrans.Z = pTemp[i2]->data.pBody[i1].Z;
						//switch x and y 


						param->calc->CoordinateTrans(
							&pStored[i2],
							pTempTrans,
							param->fAngle[i2],
							param->pSensor[i2]);
						nShade[i2] = pTemp[i2]->data.nShade[i1];
						fSpeed[i2] = pTemp[i2]->data.fSpeed[i1];
					}


					//made log start

					for (INT i2 = 0; i2 < param->nClient; i2++)
					{
						sprintf_s(bFile, 256, "[%.1f %.1f %.1f] [%.1f %.1f %.1f] (%d %.1f)|",
							pTemp[i2]->data.pBody[i1].Y,
							pTemp[i2]->data.pBody[i1].X,
							pTemp[i2]->data.pBody[i1].Z,
							pStored[i2].X,
							pStored[i2].Y,
							pStored[i2].Z,
							nShade[i2],
							fSpeed[i2]);

						WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);
					}

					sprintf_s(bFile, 256, "\n");
					WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);

					//made log end

					LONG nAvailData = PickAvailData(pStored, nShade, fSpeed, param->nClient);

					//debugging
					//nAvailData = param->nClient;


					param->calc->InitiliazeSolute(
						nAvailData,
						pStored,
						&pBodyCurrent[i1]);

					param->calc->NonlinearSolute(
						nAvailData,
						param->pSensor, 
						pStored, 
						&pBodyCurrent[i1]);
					
				}
				bCalc = TRUE;

				param->vision->DrawHuman(pBodyCurrent);

				//made log start


				sprintf_s(bFile, 256, "FinalResult:\n");
				WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);

				for (INT i1 = 0; i1 < BODY_LENGTH; i1++)
				{
					sprintf_s(bFile, 256, "%.1f %.1f %.1f\n",
						pBodyCurrent[i1].X,
						pBodyCurrent[i1].Y,
						pBodyCurrent[i1].Z);
					    
					WriteFile(fLogController, bFile, strlen(bFile), &nWrite, NULL);
				}
				//made log end

				

			}
			
		}
		//calc

		for (INT i1 = 0; i1 < param->nClient; i1++)
			ClientState(
				i1, 
				(BOOL)(nConnected >> i1 & 1), 
				*param->nCalibrationState, 
				param->sever->GetSenorPos() + i1, 
				param->pSensor + i1 ,
				param->sever->GetDelay(i1),
				param->hwnd);

		ShowSeverTime(param);





		
		param->hrc->HighResolutionSleep(30, &nThreadTimeStamp);
		//paint

	}
}

//pick available joint position data by nShade and fSpeed

INT Controller::PickAvailData(CameraSpacePoint *pStored, INT *nShade, DOUBLE *fSpeed, const INT nClient)
{
	// calc the "proper" speed

	INT nProper = 0;
	DOUBLE fAverSpeed = 0;
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		if (*(nShade + i1) == 0)
		{
			fAverSpeed += *(fSpeed + i1);
			nProper++;
		}
	}
	if (nProper>0) fAverSpeed /= (DOUBLE)nProper;
	
	DOUBLE fDiffSpeed[MAX_SENSOR];
	memset(fDiffSpeed, 0, sizeof(fDiffSpeed));
	if (nProper > 0)
	{
		for (INT i1 = 0; i1 < nClient; i1++)
		{
			fDiffSpeed[i1] = abs(*(fSpeed + i1) - fAverSpeed);
		}
	}

	// sort the data by the nShade with high to low
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		INT nNext = i1;
		for (INT i2 = i1 + 1; i2 < nClient;i2++)
			if (*(nShade + i2) > *(nShade+nNext) ||
				(*(nShade + i2) == *(nShade + nNext) && fDiffSpeed[i2]<fDiffSpeed[nNext]))
			{
				nNext = i2;
			}

		CameraSpacePoint pTemp = *(pStored + nNext);
		*(pStored + nNext) = *(pStored + i1);
		*(pStored + i1) = pTemp;

		INT nTemp = *(nShade + nNext);
		*(nShade + nNext) = *(nShade + i1);
		*(nShade + i1) = nTemp;


		DOUBLE fTemp = *(fSpeed + nNext);
		*(fSpeed + nNext) = *(fSpeed + i1);
		*(fSpeed + i1) = fTemp;
	}
	//pick available data
	INT nAvail = 0;
	while (*(nShade + nAvail) == 0 || nAvail<round((DOUBLE)(nClient) / 2))
	{
		nAvail++;
	}

	//how to balance the shade factor and the speed factor?


	return(nAvail);
}


void Controller::ClientState(INT nClientIndex, BOOL bConnect, INT nCalibration, CameraSpacePoint *pCurrentSensor, CameraSpacePoint *pSensor, INT nDelay, HWND *hwnd)
{
	HDC hdc;
	WCHAR wstrClientState[70], wstrConnect[35], wstrCalibration[35];
	RECT rect;
	HBRUSH hBrush;
	//init str start
	if (bConnect)
	{
		swprintf_s(wstrConnect, 35, TEXT("Delay :%dms"),nDelay);
		if (nCalibration == 2)
			swprintf_s(wstrCalibration, 35, TEXT("(%.1f %.1f %.1f)"),
				pSensor->X,
				pSensor->Y,
				pSensor->Z);
		else
			swprintf_s(wstrCalibration, 35, TEXT("(%.1f %.1f %.1f)"),
				pCurrentSensor->X,
				pCurrentSensor->Y,
				pCurrentSensor->Z);
	}
	else
	{
		swprintf_s(wstrConnect, 35, TEXT("Disconnected"));
		wstrCalibration[0] = '\0';
	}
	

	//init str end
	//start paint
	hdc = GetDC(*hwnd);
	
	SetRect(&rect, 10, 25 + nClientIndex * 25, 290, 20 + (nClientIndex + 1) * 25);
	hBrush = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hdc, &rect, hBrush);
	swprintf_s(wstrClientState, 70, TEXT("Client#%d  %s %s"), nClientIndex, wstrConnect, wstrCalibration);
	TextOut(hdc, 20, 25 + nClientIndex * 25, wstrClientState, wcslen(wstrClientState));

	//end paint
	ReleaseDC(*hwnd, hdc);
}

void Controller::ClientResult(HWND *hwnd, INT nJointIndex, CameraSpacePoint *pJoint)
{
	HDC hdc;
	WCHAR wstrOutput[75];
	RECT rect;
	HBRUSH hBrush;
	
	swprintf_s(
		wstrOutput, 
		75, 
		TEXT("Joint No.%d [%.1f %.1f %.1f]\n"),
		nJointIndex,
		pJoint->X,
		pJoint->Y,
		pJoint->Z);

	//init str end
	//start paint
	hdc = GetDC(*hwnd);

	SetRect(&rect, 310, 50 + nJointIndex * 20, 750, 50 + (nJointIndex + 1) * 20);
	hBrush = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hdc, &rect, hBrush);
	TextOut(hdc, 310, 50 + nJointIndex * 20, wstrOutput, wcslen(wstrOutput));

	//end paint
	ReleaseDC(*hwnd, hdc);
}

void Controller::ShowSeverTime(ThreadParamController *param)
{
	HDC hdc;
	WCHAR wstrOutput[50];
	RECT rect;
	HBRUSH hBrush;

	swprintf_s(
		wstrOutput,
		50,
		TEXT("Sever Time: %dms"), 
		param->hrc->HighResolutionTime());

	hdc = GetDC(*param->hwnd);

	SetRect(&rect, 310, 25, 750, 50);
	hBrush = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hdc, &rect, hBrush);

	TextOut(hdc, 310, 25, wstrOutput, wcslen(wstrOutput));

	ReleaseDC(*param->hwnd, hdc);
}

BOOL Controller::ButtonClick_Calibration(DOUBLE fDistance)
{
	if (param.sever->Calibration() != (1 << nClient) - 1)
		return(FALSE);

	/*
	DOUBLE fDiff[MAX_SENSOR];
	param.sever->CalibrationProcess(pTmpSensor[nCalibrationState], fDiff);
	
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		CameraSpacePoint pTmpCurrent;
		param.calc->InitialCoordinateTrans_ver2(&fAngle[i1], CROSSING_MARK_LENGTH, fDiff[i1]);
		pTmpCurrent.X = -pTmpSensor[0][i1].X;
		pTmpCurrent.Y = -pTmpSensor[0][i1].Y;
		pTmpCurrent.Z = -pTmpSensor[0][i1].Z;
		param.calc->CoordinateTrans(&pSensor[i1], pTmpCurrent, fAngle[i1]);
	}
	param.sever->EndCalibration();
	*/
	for (INT i1 = 0; i1 < nClient; i1++)
		pTmpSensor[nCalibrationState][i1] = *(param.sever->GetSenorPos() + i1);
	nCalibrationState++;
	CameraSpacePoint pTmpCurrent;
	if (nCalibrationState == 2)
	{
		for (INT i1 = 0; i1 < nClient; i1++)
		{
			param.calc->InitialCoordinateTrans(&fAngle[i1], fDistance, pTmpSensor[0][i1], pTmpSensor[1][i1]);

			pTmpCurrent.X = -pTmpSensor[0][i1].X;
			pTmpCurrent.Y = -pTmpSensor[0][i1].Y;
			pTmpCurrent.Z = -pTmpSensor[0][i1].Z;

			param.calc->CoordinateTrans(&pSensor[i1], pTmpCurrent, fAngle[i1]);
		}
		param.sever->EndCalibration();
	}

	return(TRUE);
}



INT Controller::CalibrationStep()
{
	return(nCalibrationInterface);
}

void Controller::CalibrationInterfaceStepUp()
{
	nCalibrationInterface++;
}
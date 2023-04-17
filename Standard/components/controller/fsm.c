/**
 * @file fsm.c/h
 * @author ���廪
 * @brief ״̬��ʵ�ֺ���
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "fsm.h"
#include "main.h"

/**
 * @brief           ע��״̬��
 * @param[out]      pFsm: ״̬���ṹ����ָ��ָ��
 * @param[in]       pTable: ״̬��ָ��
 */
void FSM_Regist(FSM *pFsm, FsmTable *pTable)
{
    pFsm->pFsmTable = pTable;
}

/*״̬Ǩ��*/
/**
 * @brief           ״̬Ǩ��
 * @param[out]      pFsm: ״̬���ṹ����ָ��ָ��
 * @param[in]       state: ״̬
 */
void FSM_StateTransfer(FSM *pFsm, int state)
{
    pFsm->transfer_flag = 0;
    pFsm->curState = state;
}

/**
 * @brief           �¼�����
 * @param[out]      pFsm: ״̬���ṹ����ָ��ָ��
 * @param[in]       event: �¼�
 */
void FSM_EventHandle(FSM *pFsm, int event)
{
    FsmTable *pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = NULL; //����ָ���ʼ��Ϊ��
    int NextState;
    int CurState = pFsm->curState;
    int g_max_num = pFsm->size;
    int flag = 0; //��ʶ�Ƿ���������
    int Transfer_flag;
    int i;

    /*��ȡ��ǰ��������*/
    for (i = 0; i < g_max_num; i++)
    {
        //���ҽ�����ǰ״̬������ָ�����¼����Ҳ�ִ����
        if (event == pActTable[i].event && CurState == pActTable[i].CurState)
        {
            flag = 1;
            eventActFun = pActTable[i].eventActFun;
            NextState = pActTable[i].NextState;
            break;
        }
    }

    if (flag) //�������������
    {
        /*����ִ��*/
        if (eventActFun)
        {
            eventActFun();
        }

        Transfer_flag = pFsm->transfer_flag;
        //��ת����һ��״̬
        if (Transfer_flag == 1)
        {
            FSM_StateTransfer(pFsm, NextState);
        }
    }
}

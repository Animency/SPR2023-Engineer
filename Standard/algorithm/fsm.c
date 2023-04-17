#include "algorithm.h"

/*状态机注册,给它一个状态表*/
void FSM_Regist(FSM *pFsm, FsmTable *pTable)
{
    pFsm->pFsmTable = pTable;
}

/*状态迁移*/
void FSM_StateTransfer(FSM *pFsm, int state)
{
    pFsm->transfer_flag = 0;
    pFsm->curState = state;
}

/*事件处理*/
void FSM_EventHandle(FSM *pFsm, int event)
{
    FsmTable *pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = NULL; //函数指针初始化为空
    int NextState;
    int CurState = pFsm->curState;
    int g_max_num = pFsm->size;
    int flag = 0; //标识是否满足条件
    int Transfer_flag;
    int i;

    /*获取当前动作函数*/
    for (i = 0; i < g_max_num; i++)
    {
        //当且仅当当前状态下来个指定的事件，我才执行它
        if (event == pActTable[i].event && CurState == pActTable[i].CurState)
        {
            flag = 1;
            eventActFun = pActTable[i].eventActFun;
            NextState = pActTable[i].NextState;
            break;
        }
    }

    if (flag) //如果满足条件了
    {
        /*动作执行*/
        if (eventActFun)
        {
            eventActFun();
        }

        Transfer_flag = pFsm->transfer_flag;
        //跳转到下一个状态
        if (Transfer_flag == 1)
        {
            FSM_StateTransfer(pFsm, NextState);
        }
    }
}

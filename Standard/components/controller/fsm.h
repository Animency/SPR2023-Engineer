/**
 * @file fsm.c/h
 * @author 何清华
 * @brief 状态机实现函数
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#ifndef FSM_H
#define FSM_H

typedef struct 
{
    int CurState;          //当前状态
    int event;             //事件
    int NextState;         //下一个状态
    void (*eventActFun)(); //函数指针
} FsmTable;

/*状态机类型*/
typedef struct
{
    int curState;          //当前状态
    FsmTable *pFsmTable; //状态表
    int size;              //表的项数
    int transfer_flag;
} FSM;

/**
 * @brief           注册状态机
 *
 * @param[out]      pFsm: 状态机结构数据指针指针
 * @param[in]       pTable: 状态表指针
 */
extern void FSM_Regist(FSM *pFsm, FsmTable *pTable);

/**
 * @brief           状态迁移
 *
 * @param[out]      pFsm: 状态机结构数据指针指针
 * @param[in]       state: 状态
 */
extern void FSM_StateTransfer(FSM *pFsm, int state);

/**
 * @brief           事件处理
 *
 * @param[out]      pFsm: 状态机结构数据指针指针
 * @param[in]       event: 事件
 */
extern void FSM_EventHandle(FSM *pFsm, int event);

#endif

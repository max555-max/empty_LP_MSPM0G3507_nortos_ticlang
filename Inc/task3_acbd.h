#ifndef __TASK3_ACBD_H_
#define __TASK3_ACBD_H_

/*
 * 第三问任务入口
 *
 * 当前采用的策略：
 *   上电前手动把小车从 A 点对准 C 点；
 *   A -> C：锁定初始 yaw，编码器定距直行；
 *   C -> B：八路灰度循迹右半圆；
 *   B -> D：转向对准 D 后，编码器定距直行；
 *   D -> A：八路灰度循迹左半圆，到 A 后丢线停车。
 *
 * main() 中只需要调用 task3_acbd_run()。
 */
void task3_acbd_run(void);

#endif

#ifndef __TASK2_ABCD_H_
#define __TASK2_ABCD_H_

/*
 * 第二问任务入口
 *
 * 路线：
 *   A -> B：编码器定距直行 + 陀螺仪角度保持；
 *   B -> C：八路灰度循迹右半圆；
 *   C -> D：编码器定距直行 + 陀螺仪角度保持；
 *   D -> A：八路灰度循迹左半圆，到 A 后丢线停车。
 *
 * main() 中只需要调用 task2_abcd_run()。
 */
void task2_abcd_run(void);

#endif

#ifndef __UART_CMD_H_
#define __UART_CMD_H_

/*
 * 串口命令解析模块
 *
 * 用于上位机发送目标速度、PID 参数等命令。
 * 接收中断只负责收字节，真正解析在 uart_cmd_process() 中进行，
 * 避免中断里做复杂字符串处理。
 */

/* 初始化串口命令缓冲区状态。 */
void uart_cmd_init(void);

/* 在主循环中调用，解析已经收到的一行命令。 */
void uart_cmd_process(void);

/* UART 接收中断处理函数：收集字符到命令缓冲区。 */
void uart_cmd_irq_handler(void);

#endif

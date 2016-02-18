class MotionControl
/*!
 * \brief 运动控制器类
 *          1、核心是TPlanBaseVelocity()进行T形速度规划,计算出位移曲线的分段函数GetInterpolation(int index)
 *			2、运动指令
 *          2.1、运动指令MoveAbs()和Move()
			2.2、调用终点设置函数SetEndPos()，也会导致生成一个新的运动指令，实现对终点的跟随
 *			3、指令位置的刷新与获取
 *          3.1、信号CastCurrentPos(QVector<qreal>)每T时间对外广播每个伺服周期的指令位置
 *          3.2、亦可不使用signal，外部调用update()更新一次伺服位置后再调用GetCurrentPos()得到当前伺服周期的位置
 */
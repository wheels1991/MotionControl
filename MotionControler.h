#ifndef MOTIONCONTROLER_H
#define MOTIONCONTROLER_H

#include <QVector>
#include <QObject>

class QTimer;
/*!
 * \brief 运动控制器类
 *          1、核心是TPlanBaseVelocity()进行T形速度规划,计算出位移曲线的分段函数GetInterpolation(int index),
 *          2、运动指令MoveAbs()和Move()
 *          3.1、信号CastCurrentPos(QVector<qreal>)每T时间对外广播每个伺服周期的指令位置
 *          3.2、亦可不使用signal，外部调用update()更新一次伺服位置后再调用GetCurrentPos()得到当前伺服周期的位置
 */
class MotionControler : QObject
{
public:
    /* 运动状态枚举 */
    enum motionStatus{
        moveAbs,
        move,
        free,
    };
    enum curveType{
        type0,  //加速-减速
        type1,  //加速度-匀速-减速
        type2,  //减速-加速度
        type3,  //减速-匀速-加速
    };

    MotionControler();

    /* 运动指令 */
    void MoveAbs(QVector<qreal> pos);
    void MoveAbs(qreal pos, int axis);
    void Move(QVector<qreal> pos);
    void Move(qreal pos, int axis);
    void RapidStop();
    void Stop();

    /* 运动规划 */
    QVector<qreal> TPlanBaseTime(qreal time);
    bool TPlanBaseVelocity();
    void Update();
    qreal UpdateRemainPos();
    QVector<qreal> UpdateCurrentPos();
    qreal GetInterpolation(int index);
    inline motionStatus GetStatus() {return status;}

    /* 更改以下运动参数，可能会重新进行运动规划 */
    void SetEndPos(QVector<qreal> pos);
    void SetEndPos(qreal pos, int axis);
    void SetVelocityLimitTCP(qreal vel);
    void SetAccelorationLimitTCP(qreal acc);
    //未定义、未使用
    void SetVelocityLimit(QVector<qreal> vel);
    void SetAccelorationLimit(QVector<qreal> acc);

    /* 参数设置 */
    inline void SetCurrentPos(QVector<qreal> pos) {currentPos = pos;}
    inline void SetCurrentPos(qreal pos, int axis) {currentPos[axis] = pos;}

    inline void SetEndVelocityTCP(qreal vel) {endVelocityTCP = vel;}
    inline void SetEndVelocity(QVector<qreal> vel) {endVelocity = vel;}
    inline void SetEndVelocity(qreal vel, int axis) {endVelocity[axis] = vel;}
    inline void SetT(qreal t) { T = t;}
    inline void SetError(qreal err) {error = err;}                               //最小建议值为accelorationLimitTCP * T * T
    inline void SetAxisNum(int num) {axisNum = num;}

    /* 定义轴位置 */
    void DefPos(qreal pos, int axis);
    void DefPos(QVector<qreal> pos);

    /* 参数获取 */
    QVector<qreal> GetCurrentPos() {return currentPos;}
    qreal GetCurrentPos(int axis) {return currentPos[axis];}
    QVector<qreal> GetCurrentVel() {return currentVelocity;}
    qreal GetCurrentVelTCP() {return currentVelocityTCP;}
    QVector<qreal> GetCurrentAcc() {return currentAcceloration;}
    qreal GetCurrentAccTCP() {return currentAccelorationTCP;}


signals:
    void CastCurrentPos(QVector<qreal> pos);                                    /* 向外广播伺服位置 */
//private slots:
public:
    void SendCurrentPos();                                                      /* 由定时器每1ms触发一次 */

private:
    int axisNum;                                                                /* 轴个数 axisNum + 1 */

    QVector<qreal> currentPos;                                                  /* 各轴当前位置 */
    qreal currentVelocityTCP;                                                   /* 当前的TCP速度 */
    QVector<qreal> currentVelocity;                                             /* 各轴当前速度，利用速度分解得到 */
    qreal currentAccelorationTCP;                                               /* 当前合成加速度，即TCP加速度 */
    QVector<qreal> currentAcceloration;                                         /* 各轴当前加速度，利用加速度分解得到 */

    QVector<qreal> startPos;                                                    /* 速度规划时的起始位置：每次规划前需要更新成当前位置 */
    qreal startVelocityTCP;
    QVector<qreal> endPos;                                                      /* 终点的各轴位置 */
    qreal endVelocityTCP;                                                       /* 终点的TCP速度 */
    QVector<qreal> endVelocity;                                                 /* 终点的各轴速度 */

    qreal remainPosTCP;                                                         /* 剩下的总路程 */
    QVector<qreal> remainPos;                                                   /* 剩下的各轴位移 */

    motionStatus status;                                                           /* 当前运动状态 */

    QVector<qreal> velocityLimit;                                               /* 各轴速度限制 */
    qreal velocityLimitTCP;                                                     /* TCP速度限制 */
    QVector<qreal> accelorationLimit;                                           /* 各轴加速度限制 */
    qreal accelorationLimitTCP;                                                 /* TCP加速度限制 */

    QVector<qreal> multiper;                                                      /* 轴增益 */
    QVector<qreal> origin;                                                      /* 各轴原点偏移 */

    qreal T;                                                                    /* 伺服周期 */
    /* 插补过程中的参数 */
    int interpolationCounter;                                                   /* 一次速度规划中的插值索引 */
    int interpolationNum;                                                       /* 一次速度规划中的插值点总个数 */
    int interpolationType;
    qreal t1, t2, t3;                                                           /* T形运动规划中间变量 */
    qreal V1;

    qreal error;                                                                /* 终点判定误差 */

    QTimer *timer;                                                              /* 需要在运动指令执行后开始，运动指令完成后关闭
                                                                                    status = free时关闭
                                                                                    status != free时开启 */

};

#endif // MOTIONCONTROLER_H

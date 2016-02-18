#include "MotionControler.h"
#include <qmath.h>
#include <qdebug.h>
#include <QTimer>

MotionControler::MotionControler()
{
    /* 初始状态参数配置 */
    axisNum = 1;
    currentPos = QVector<qreal>(axisNum + 1, 0);
    currentVelocity = QVector<qreal>(axisNum + 1, 0);
    currentAcceloration = QVector<qreal>(axisNum + 1, 0);
    currentVelocityTCP = 0;
    currentAccelorationTCP = 0;

    endPos = QVector<qreal>(axisNum + 1, 0);
    endVelocity = QVector<qreal>(axisNum + 1, 0);
    endVelocityTCP = 0;

    remainPos = QVector<qreal>(axisNum + 1, 0);
    remainPosTCP = 0;
    T = 1e-3;

    multiper = QVector<qreal>(axisNum + 1, 1);
    origin = QVector<qreal>(axisNum + 1, 0);
    error = 0.01;
    status = free;

    velocityLimitTCP = 0;
    accelorationLimitTCP = 0;

    interpolationCounter = 0;
    timer = new QTimer();
//    connect(timer, SIGNAL(timeout()), this, SLOT(SendCurrentPos()));
//    timer->start(1);
}
/*!
 * \brief MotionControler::TPlan    指定期望时间的T形速度规划,相对运动方式
 * \return                          插补距离
 */
QVector<qreal> MotionControler::TPlanBaseTime(qreal time)
{
    int numOfInterpolation = (int) (time / T);                                  /* 插补点个数为总时间除以插补周期 */
    QVector<qreal> pos(numOfInterpolation);                /* 定义数组长度为numOfInterpolation */
    bool succeed = false;                                                       /* 规划成功标志 */
    int j = 0;
    float delta;                                                                /* 解一元二次方程中的解的判据 */
    float V1;                                                                   /* 匀速段的速度 */
    float V[2];
    float t1;                                                                   /* 匀速段开始时间 */
    float t2;                                                                   /* 匀速段结束时间 */
    while (j <= 4){
        j++;
        /*情况1：加速度-匀速-减速     先假设后验证*/
        delta = 4 * (accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP)
                * (accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP)
                - 8 * (currentVelocityTCP * currentVelocityTCP + endVelocityTCP * endVelocityTCP + 2 * accelorationLimitTCP * remainPosTCP);
        if (delta < 0 ){
            succeed = false;
        }else if (delta == 0){
            V1 = (accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP) / 2;
            if (~(V1 >= currentVelocityTCP && V1 >= endVelocityTCP
               && qAbs(V1) <= velocityLimitTCP
               && (time - (V1 - currentVelocityTCP) / accelorationLimitTCP - (endVelocityTCP - V1) / (-accelorationLimitTCP)) >= 0))
            {
                succeed = true;             //有解
            }
        }else {
             V[0] = (2 * (accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP) + qSqrt(delta)) / 4;
             V[1] = (2 * (accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP) - qSqrt(delta)) / 4;
             if (V[0] >= currentVelocityTCP && V[0] >= endVelocityTCP
                && qAbs(V[0]) <= velocityLimitTCP
                && (time - (V[0] - currentVelocityTCP) / accelorationLimitTCP - (endVelocityTCP - V[0]) / (-accelorationLimitTCP)) >= 0)
             {
                 V1 = V[0];
                 succeed = true;            //有解
             }else if (V[1] >= currentVelocityTCP && V[1] >= endVelocityTCP
                          && qAbs(V[1]) <= velocityLimitTCP
                          && (time - (V[1] - currentVelocityTCP) / accelorationLimitTCP - (endVelocityTCP - V[1]) / (-accelorationLimitTCP)) >= 0)
                       {
                 V1 = V[1];
                 succeed = true;
             }else{
                 succeed = false;
             }
        }
        if (succeed){
            t1 = (V1 - currentVelocityTCP) / accelorationLimitTCP;
            t2 = time - (endVelocityTCP - V1) / (-accelorationLimitTCP);
            float t = 0;
            for (int i = 0; i < numOfInterpolation; i++){
                t += T;
                if (t <= t1){
                    pos[i] = currentVelocityTCP * t + 0.5 * accelorationLimitTCP * t * t;
                }else if (t > t1 && t <= t2){
                    pos[i] = (0 + currentVelocityTCP * t1 + 0.5 * accelorationLimitTCP * t1 * t1)
                             + V1 * (t - t1);
                }else{
                    pos[i] = (0 + currentVelocityTCP * t1 + 0.5 * accelorationLimitTCP * t1 * t1)
                            + V1 * (t2 - t1)
                            + V1 * (t - t2)
                            - 0.5 * accelorationLimitTCP * (t- t2) * (t - t2);
                }
            }
            break;
        }

        /*情况2：加速-匀速-加速  先假设后验证*/
        V1 = (2 * accelorationLimitTCP * remainPosTCP - endVelocityTCP * endVelocityTCP + currentVelocityTCP * currentVelocityTCP)
              / (2 * accelorationLimitTCP * time + 2 * currentVelocityTCP - 2 * endVelocityTCP);
        //succeed = true;
        if ( (V1 >= currentVelocityTCP && V1 <= endVelocityTCP && qAbs(V1) <= velocityLimitTCP
               && (time - (V1 - currentVelocityTCP) / accelorationLimitTCP - (endVelocityTCP - V1) / accelorationLimitTCP >= 0)) ){
            succeed = true;
        }
        if (succeed){
            t1 = (V1 - currentVelocityTCP) / accelorationLimitTCP;
            t2 = time - (endVelocityTCP - V1) / accelorationLimitTCP;
            float t = 0;
            for (int i = 0; i < numOfInterpolation; i++){
                t += T;
                if (t <= t1){
                    pos[i] = currentVelocityTCP * t + 0.5 * accelorationLimitTCP * t * t;
                }else if (t > t1 && t <= t2){
                    pos[i] = (0 + currentVelocityTCP * t1 + 0.5 * accelorationLimitTCP * t1 * t1)
                             + V1 * (t - t1);
                }else{
                    pos[i] = (0 + currentVelocityTCP * t1 + 0.5 * accelorationLimitTCP * t1 * t1)
                            + V1 * (t2 - t1)
                            + V1 * (t - t2)
                            + 0.5 * accelorationLimitTCP * (t- t2) * (t - t2);
                }
            }
            break;
        }

        /*情况3：减速-匀速-加速    先假设后验证*/
        delta = 4 * (-accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP)
                * (-accelorationLimitTCP * time + currentVelocityTCP + endVelocityTCP)
                - 8 * (currentVelocityTCP * currentVelocityTCP + endVelocityTCP * endVelocityTCP - 2 * accelorationLimitTCP * remainPosTCP);
        if (delta < 0){
            succeed = false;
        }else if (delta == 0){
            V1 = (currentVelocityTCP + endVelocityTCP - accelorationLimitTCP *time) / 2;
            if (~(V1 <= currentVelocityTCP && V1 <= endVelocityTCP && qAbs(V1) <= velocityLimitTCP
                  && time - (V1 - currentVelocityTCP)/(-accelorationLimitTCP) - (endVelocityTCP - V1) / accelorationLimitTCP >= 0)){
                succeed = false;
            }
        }else{
            V[0] = (2 * (currentVelocityTCP + endVelocityTCP - accelorationLimitTCP * time) + qSqrt(delta)) / 4;

            V[1] = (2 * (currentVelocityTCP + endVelocityTCP - accelorationLimitTCP * time) - qSqrt(delta)) / 4;
            if (V[0] <= currentVelocityTCP && V[0] <= endVelocityTCP && qAbs(V[0]) <= velocityLimitTCP
                  && time - (V[0] - currentVelocityTCP)/(-accelorationLimitTCP) - (endVelocityTCP - V[0]) / accelorationLimitTCP >= 0){
                V1 = V[0];
                succeed = true;
            }else if(V[1] <= currentVelocityTCP && V[1] <= endVelocityTCP && qAbs(V[1]) <= velocityLimitTCP
                     && time - (V[1] - currentVelocityTCP)/(-accelorationLimitTCP) - (endVelocityTCP - V[1]) / accelorationLimitTCP >= 0){
                V1 = V[1];
                succeed = true;
            }else{
                succeed = false;
            }
        }
        if (succeed){
            t1 = (V1 - currentVelocityTCP) / (-accelorationLimitTCP);
            t2 = time - (endVelocityTCP - V1) / accelorationLimitTCP;
            float t = 0;
            for (int i = 0; i < numOfInterpolation; i++){
                t += T;
                if (t <= t1){
                    pos[i] = currentVelocityTCP * t - 0.5 * accelorationLimitTCP * t * t;
                }else if (t > t1 && t <= t2){
                    pos[i] = (0 + currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1)
                             + V1 * (t - t1);
                }else{
                    pos[i] = (0 + currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1)
                            + V1 * (t2 - t1)
                            + V1 * (t - t2)
                            + 0.5 * accelorationLimitTCP * (t- t2) * (t - t2);
                }
            }
            break;
        }

        /*情况4：减速-匀速-减速*/
        V1 = (2 * accelorationLimitTCP * remainPosTCP + endVelocityTCP * endVelocityTCP - currentVelocityTCP * currentVelocityTCP)
              / (2 * accelorationLimitTCP * time - 2 * currentVelocityTCP + 2 * endVelocityTCP);
        //succeed = true;
        if (V1 <= currentVelocityTCP && V1 >= endVelocityTCP && qAbs(V1) <= velocityLimitTCP
               && (time - (V1 - currentVelocityTCP) / (-accelorationLimitTCP) - (endVelocityTCP - V1) / (-accelorationLimitTCP) >= 0)){
            succeed = true;
        }
        if (succeed){
            t1 = (V1 - currentVelocityTCP) / (-accelorationLimitTCP);
            t2 = time - (endVelocityTCP - V1) / (-accelorationLimitTCP);
            float t = 0;
            for (int i = 0; i < numOfInterpolation; i++){
                t += T;
                if (t <= t1){
                    pos[i] = currentVelocityTCP * t - 0.5 * accelorationLimitTCP * t * t;
                }else if (t > t1 && t <= t2){
                    pos[i] = (0 + currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1)
                             + V1 * (t - t1);
                }else{
                    pos[i] = (0 + currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1)
                            + V1 * (t2 - t1)
                            + V1 * (t - t2)
                            - 0.5 * accelorationLimitTCP * (t- t2) * (t - t2);
                }
            }
            break;
        }
    }
    if (succeed) {
        currentVelocityTCP = pos.first() / T;
        return pos;
    } else {
        return QVector<qreal>();
    }
}

/*!
 * \brief MotionControler::TPlan    指定期望速度的T形速度规划，相对运动方式
 * \return                          各轴下一个插补周期的目标位置
 */
bool MotionControler::TPlanBaseVelocity()
{
    bool succeed = false;
    /*情况1：加速度-减速*/
    if (remainPosTCP > 0
        && (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP - endVelocityTCP * endVelocityTCP) / (2 * accelorationLimitTCP) >= remainPosTCP ){
        succeed = true;
        V1 = qSqrt(accelorationLimitTCP * remainPosTCP + 0.5 * startVelocityTCP * startVelocityTCP + 0.5 * endVelocityTCP * endVelocityTCP);
        t1 = (V1 - startVelocityTCP) / accelorationLimitTCP;
        t2 = (2 * V1 - startVelocityTCP - endVelocityTCP) / accelorationLimitTCP;
        interpolationNum = (int) (t2 / T) - 1;
        interpolationType = type0;
    }
    /* 情况2： 加速-匀速-减速*/
    else if (remainPosTCP > 0 && (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP - endVelocityTCP * endVelocityTCP) / (2 * accelorationLimitTCP) < remainPosTCP ){
        succeed = true;
        t1 = (velocityLimitTCP - startVelocityTCP) / accelorationLimitTCP;
        t2 = t1 + (remainPosTCP -
                         (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP - endVelocityTCP * endVelocityTCP)
                         / (2 * accelorationLimitTCP)) / velocityLimitTCP;
        t3 = t2 + (velocityLimitTCP - endVelocityTCP) / accelorationLimitTCP;
        interpolationNum = (int) (t3 / T) - 1;
        interpolationType = type1;
    }
    /* 情况3： 减速-加速*/
    else if (remainPosTCP < 0 && (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP -endVelocityTCP * endVelocityTCP) / (2 * accelorationLimitTCP) > (-remainPosTCP)) {
        succeed = true;
        V1 = -qSqrt(-accelorationLimitTCP * remainPosTCP - 0.5 * startVelocityTCP * startVelocityTCP -0.5 * endVelocityTCP * endVelocityTCP);
        t1 = (V1 - startVelocityTCP) / (-accelorationLimitTCP);
        t2 = (-2 * V1 + startVelocityTCP + endVelocityTCP) / accelorationLimitTCP;
        interpolationNum = (int) (t2 / T) - 1;
        interpolationType = type2;
    }

    /*情况4：减速-匀速-加速*/
    else if (remainPosTCP < 0 && (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP -endVelocityTCP * endVelocityTCP) / (2 * accelorationLimitTCP) <= (-remainPosTCP)) {
        succeed = true;
        float t1 = (startVelocityTCP + velocityLimitTCP) / accelorationLimitTCP;
        float t2 = t1 + (remainPosTCP +
                         (2 * velocityLimitTCP * velocityLimitTCP - startVelocityTCP * startVelocityTCP - endVelocityTCP * endVelocityTCP)
                         / (2 * accelorationLimitTCP)) / (-velocityLimitTCP);
        float t3 = t2 + (endVelocityTCP + velocityLimitTCP) / accelorationLimitTCP;
        interpolationNum = (int) (t3 / T) - 1;
        interpolationType = type3;
    }
    /* 其它情况：无法规划*/
    else {
        succeed = false;
    }
    return succeed;
}

/*!
 * \brief MotionControler::Update   更新运动控制器各状态参数
 *                                     1、每个伺服周期调用一次
 *                                     2、进行运动规划
 */
void MotionControler::Update()
{
    if (status != free) {
        qreal interpolation;
        if (interpolationCounter == 0) {                                        /* 首次规划 */
            UpdateRemainPos();
            TPlanBaseVelocity();
            interpolation = GetInterpolation(interpolationCounter);
        }
        if (interpolationCounter <= interpolationNum && interpolationCounter >= 1 && status != free) {
            interpolation = GetInterpolation(interpolationCounter)
                            - GetInterpolation(interpolationCounter - 1);       /* 插补距离 */
        } else {
            interpolation = 0;
        }
        UpdateCurrentPos();
        UpdateRemainPos();
        currentVelocityTCP = interpolation / T;                                 /* 更新当前速度 */
        qDebug() << interpolationCounter;
        qDebug() << "remainPosTCP: " << remainPosTCP;
        qDebug() << "interpolationTable: " << GetInterpolation(interpolationCounter);
        qDebug() << "currentPos: " << (UpdateCurrentPos().isEmpty() ? QVector<qreal>(axisNum + 1, 0) : UpdateCurrentPos());
        interpolationCounter++;
        if (interpolationCounter > interpolationNum){
            status = free;
            timer->stop();
        }
    }
}
/*!
 * \brief MotionControler::UpdateCurrentPos 更新当前位置
 */
QVector<qreal> MotionControler::UpdateCurrentPos()
{
    qreal remainTotal = 0;
    for (int index = 0; index <= axisNum; index++) {
        remainTotal += qPow(endPos[index] - startPos[index], 2);
    }
    remainTotal = qSqrt(remainTotal);
    if (qAbs(remainTotal) < error) {                                            /* 说明已经到达目标位置 */
        return QVector<qreal>();
    }
    qreal coeficiency = GetInterpolation(interpolationCounter) / remainTotal;
    for (int index = 0; index <= axisNum; index++) {
        qreal remain = endPos[index] - startPos[index];
        currentPos[index] = startPos[index] + coeficiency * remain;
    }
    QVector<qreal> pos(axisNum + 1);
    for (int index = 0; index <= axisNum; index++){
        pos[index] = multiper[index] * (currentPos[index] + origin[index]);                         /* 加上原点偏移 */
    }
    return pos;
}
/*!
 * \brief MotionControler::GetInterpolation 获取插补点
 * \param index
 * \return
 */
qreal MotionControler::GetInterpolation(int index)
{
    index = qBound<int>(0, index, interpolationNum);
    qreal t = (index + 1) * T;
    switch (interpolationType) {
    case type0:
            if (t <= t1){
                return currentVelocityTCP * t + 0.5 * accelorationLimitTCP * t * t;
            }else{
                return currentVelocityTCP * t1 + 0.5 * accelorationLimitTCP * t1 * t1
                         + V1 * (t - t1)
                         - 0.5 * accelorationLimitTCP * (t - t1) * (t - t1);
            }
        break;
    case type1:
        if (t <= t1){
            return currentVelocityTCP * t + 0.5 * accelorationLimitTCP * t * t;
        }else if (t > t1 && t <= t2) {
            return currentVelocityTCP * t1+ 0.5 * accelorationLimitTCP * t1 * t1
                     + velocityLimitTCP * (t - t1);
        }else {
            return currentVelocityTCP * t1+ 0.5 * accelorationLimitTCP * t1 * t1
                     + velocityLimitTCP * (t2 - t1)
                     + velocityLimitTCP * (t - t2) - 0.5 * accelorationLimitTCP * (t - t2) * (t - t2);
        }
        break;
    case type2:
        if (t <= t1){
            return currentVelocityTCP * t - 0.5 * accelorationLimitTCP * t * t;
        }else {
            return currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1
                     + V1 * (t - t1) + 0.5 * accelorationLimitTCP * (t - t1) * (t - t1);
        }
        break;
    case type3:
        if (t <= t1){
            return currentVelocityTCP * t - 0.5 * accelorationLimitTCP * t * t;
        }else if (t > t1 && t <= t2){
            return currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1
                     - velocityLimitTCP * (t - t1);
        }else {
            return currentVelocityTCP * t1 - 0.5 * accelorationLimitTCP * t1 * t1
                     - velocityLimitTCP * (t2 - t1)
                     - velocityLimitTCP * (t - t2) + 0.5 * accelorationLimitTCP * (t - t2) * (t - t2);
        }
        break;
    default:
        break;
    }
    return 0;
}

void MotionControler::MoveAbs(QVector<qreal> pos)
{
    if (status == free) {                                                        /* 取消if条件后可以运动中更改运动指令 */
        SetEndPos(pos);
        status = moveAbs;
        timer->start((int)T);
    }
}

void MotionControler::MoveAbs(qreal pos, int axis)
{
    if (status == free){
        SetEndPos(pos, axis);
        status = moveAbs;
        timer->start((int)T);
    }
}

void MotionControler::Move(QVector<qreal> pos)
{
    for (int index = 0; index <= axisNum; index++) {
        pos[index] += currentPos[index];
    }
    MoveAbs(pos);
    status = move;
}

void MotionControler::Move(qreal pos, int axis)
{
    pos += currentPos[axis];
    MoveAbs(pos, axis);
    status = move;
}
/*!
 * \brief MotionControler::RapidStop    快速停止，加速度无限大
 *                                      直接停在当前位置
 */
void MotionControler::RapidStop()
{
    status = free;
    timer->stop();
}
/*!
 * \brief MotionControler::Stop     以最大加速度减速直到停止
 */
void MotionControler::Stop()
{
    //计算出以最大加速度减速后停下来的位置，将其设置成endPos,执行move指令
    qreal stopDistance = qPow(currentVelocityTCP, 2) / (2 * accelorationLimitTCP);
    qreal coefficiency = stopDistance / UpdateRemainPos();
    QVector<qreal> stopPos(axisNum + 1);
    for (int index = 0; index <= axisNum; index++) {
        stopPos[index] = currentPos[index] + coefficiency * (endPos[index] - currentPos[index]);
    }
    SetEndPos(stopPos);                                                         /* 重置终点 */
}
/*!
 * \brief MotionControler::DefPos   重定义当前轴位置
 */
void MotionControler::DefPos(qreal pos, int axis)
{
    origin[axis] = currentPos[axis] + origin[axis] - pos;
    currentPos[axis] = pos;
}

void MotionControler::DefPos(QVector<qreal> pos)
{
    for (int index = 0; index <= axisNum; index++){
        origin[index] = currentPos[index] + origin[index] - pos[index];
        currentPos[index] = pos[index];
    }
}
/*!
 * \brief MotionControler::UpdateRemainPos  更新剩余距离
 * \return                                  返回剩余距离
 */
qreal MotionControler::UpdateRemainPos()
{
    qreal remain = 0;
    for (int index = 0; index <= axisNum; index++) {
        remainPos[index] = endPos[index] - currentPos[index];
        remain += qPow(remainPos[index], 2);
    }
    remainPosTCP = qSqrt(remain);
    return remainPosTCP;
}
/*!
 * \brief MotionControler::SetEndPos    设置终点，会导致重新进行运动规划
 * \param pos
 */
void MotionControler::SetEndPos(QVector<qreal> pos)
{
    if (endPos == pos){
        return;
    } else {
        status = moveAbs;                                                        /* 当MType != free时才会进行运动规划 */
        endPos = pos;
        interpolationCounter = 0;
//        interpolationTable = QVector<qreal>();                                  /* 重置插值列表 */
        startPos = currentPos;
        startVelocityTCP = currentVelocityTCP;
    }
}

void MotionControler::SetEndPos(qreal pos, int axis)
{
    if (endPos[axis] == pos) {
        return;
    } else {
        status = moveAbs;
        endPos[axis] = pos;
        interpolationCounter = 0;
//        interpolationTable = QVector<qreal>();
        startPos = currentPos;
        startVelocityTCP = currentVelocityTCP;

    }
}
/*!
 * \brief MotionControler::SetVelocityLimitTCP  重置期望速度
 *                                              1、如果MType = free，则不会带来运动规划
 *                                              2、如果MType != free，则会重新进行运动规划
 * \param vel
 */
void MotionControler::SetVelocityLimitTCP(qreal vel)
{
    if (velocityLimitTCP == vel) {
        return;
    } else {
        velocityLimitTCP = vel;
        interpolationCounter = 0;
//        interpolationTable = QVector<qreal>();
        startPos = currentPos;
        startVelocityTCP = currentVelocityTCP;

    }
}
/*!
 * \brief MotionControler::SetAccelorationLimitTCP  重置期望加速度
 *                                                  1、如果MType = free，则不会带来运动规划
 *                                                  2、如果MType != free，则会重新进行运动规划
 *                                                  3、更新误差限
 * \param acc
 */
void MotionControler::SetAccelorationLimitTCP(qreal acc)
{
    if (accelorationLimitTCP == acc) {
        return;
    } else {
        accelorationLimitTCP = acc;
        interpolationCounter = 0;
//        interpolationTable = QVector<qreal>();
        startPos = currentPos;
        startVelocityTCP = currentVelocityTCP;
        error = accelorationLimitTCP * qPow(T, 2);
    }
}
/*!
 * \brief MotionControler::sendCurrentPos   slot,由定时器触发，
 *                                          用来激活castCurrentPos()信号，广播伺服位置
 */
void MotionControler::SendCurrentPos()
{
    Update();
    if (status != free) {
//        emit CastCurrentPos(UpdateCurrentPos());
    }
}

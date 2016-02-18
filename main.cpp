#include <QCoreApplication>
#include <MotionControler.h>
#include <QDebug.h>
#include <qmath.h>
#include <QFile>

int main(int argc, char *argv[])
{
//    QCoreApplication a(argc, argv);

    MotionControler mc;
//    状态配置
    mc.SetAccelorationLimitTCP(50000);
    mc.SetVelocityLimitTCP(80);
    mc.SetEndVelocityTCP(0);
    mc.SetError(500 * 1e-6);
    int axisNum = 1;
    QVector<qreal> endPos(axisNum + 1);
    for (int index = 0; index <= axisNum; index++) {
        endPos[index] = index + 1;
    }
    mc.MoveAbs(endPos);
    int count = 0;
    QString path("motionPlan.csv");
    QFile csvFile(path);
    if(!csvFile.open(QIODevice::WriteOnly)){
        return 0;
    }
    QTextStream csv(&csvFile);
    while (1) {
        if (mc.GetStatus() == MotionControler::free) {
            break;
            qDebug() << "test";
        }
        mc.SendCurrentPos();
//        csv << mc.UpdateRemainPos() << endl;
        count++;
        if (count == 10){
            mc.SetAccelorationLimitTCP(5000);
//            break;
//            mc.MoveAbs(QVector<qreal>(mc.axisNum + 1, 1));
        }

    }
    qDebug() << endl;
    qDebug() << "total interpolation points number: " << count;
    qDebug() << "finish to motion planning";
    qDebug() << endl;
//    return a.exec();
    return 0;
}


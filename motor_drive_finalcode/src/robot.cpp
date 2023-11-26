#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "robot.h"
#include "plan.h"

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
namespace robot
{

//Tcurve
auto TcurveDrive::prepareNrt()->void
{
    cef_ = doubleParam("coefficient");
//  aaa = doubleParam("coooo");// 参数构建

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto TcurveDrive::executeRT()->int //进入实时线程
{
    static double begin_angle[3];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

        this->master()->logFileRawName("TestMvj");//建立记录数据的文件夹
    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(5,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * cef_  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualCur() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualToq() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[0].actualVel() <<std::endl;
//    return 5000-count(); //运行时间为5秒
    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto TcurveDrive::collectNrt()->void {}
TcurveDrive::TcurveDrive(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_mvj\">"
       "	<GroupParam>"
       "	<Param name=\"coefficient\" default=\"1\" abbreviation=\"k\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
TcurveDrive::~TcurveDrive() = default;  //析构函数























//Tcurve_1
auto TcurveDrive1::prepareNrt()->void
{
    x_ = doubleParam("coefficientx");
    y_ = doubleParam("coefficienty");
    z_ = doubleParam("coefficientz");
//  aaa = doubleParam("coooo");// 参数构建

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto TcurveDrive1::executeRT()->int //进入实时线程
{
    static double begin_angle[3];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();
        begin_angle[1] = controller()->motorPool()[1].actualPos();
        begin_angle[2] = controller()->motorPool()[2].actualPos();

        this->master()->logFileRawName("tc1");//建立记录数据的文件夹
    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次
    double angle1 = begin_angle[1] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次
    double angle2 = begin_angle[2] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);
    controller()->motorPool()[1].setTargetPos(angle1);
    controller()->motorPool()[2].setTargetPos(angle2);

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualCur() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualToq() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[1].actualPos() <<"\t";
    lout() << controller()->motorPool()[2].actualPos() <<std::endl;
//    return 5000-count(); //运行时间为5秒
    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto TcurveDrive1::collectNrt()->void {}
TcurveDrive1::TcurveDrive1(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"move1\">"
       "	<GroupParam>"
       "	<Param name=\"coefficientx\" default=\"0\" abbreviation=\"x\"/>"
       "	<Param name=\"coefficienty\" default=\"0\" abbreviation=\"y\"/>"
       "	<Param name=\"coefficientz\" default=\"0\" abbreviation=\"z\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
TcurveDrive1::~TcurveDrive1() = default;  //析构函数



//Tcurve_2
auto BackZero::prepareNrt()->void
{

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto BackZero::executeRT()->int //进入实时线程
{
    static double begin_angle[3];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();
        begin_angle[1] = controller()->motorPool()[1].actualPos();
        begin_angle[2] = controller()->motorPool()[2].actualPos();

        this->master()->logFileRawName("tc3");//建立记录数据的文件夹
    }


    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] *(1.0-s1.getTCurve(count())) ;//count()为循环次数计数，每1ms计数一次
    double angle1 = begin_angle[1] *(1.0-s1.getTCurve(count())) ;//count()为循环次数计数，每1ms计数一次
    double angle2 = begin_angle[2] *(1.0-s1.getTCurve(count()))  ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);
    controller()->motorPool()[1].setTargetPos(angle1);
    controller()->motorPool()[2].setTargetPos(angle2);

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualCur() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualToq() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[1].actualPos() <<"\t";
    lout() << controller()->motorPool()[2].actualPos() <<std::endl;
//    return 5000-count(); //运行时间为5秒
    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto BackZero::collectNrt()->void {}
BackZero::BackZero(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"zero\">"
       "	<GroupParam>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
BackZero::~BackZero() = default;  //析构函数




//Tcurve_3
auto Rect::prepareNrt()->void
{
    x = doubleParam("x");
    y = doubleParam("y");
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Rect::executeRT()->int //进入实时线程
{

    static double begin_angle[3];
    double angle0 = 0;
    double angle1 = 0;
    double angle2 = 0;

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();
        begin_angle[1] = controller()->motorPool()[1].actualPos();
        begin_angle[2] = controller()->motorPool()[2].actualPos();

        this->master()->logFileRawName("tc2");//建立记录数据的文件夹
    }


    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double time = s1.getTc()*1000;
    if(count() <= s1.getTc()*1000){
           angle0 = begin_angle[0] + 2*PI * (x/72.0)  * s1.getTCurve(count()) ;
           controller()->motorPool()[0].setTargetPos(angle0);
       }
    if( ((s1.getTc()*1000) <= count()) && (count() <= (2*s1.getTc()*1000))){
           mout() << std::hex << 1 << std::endl;
           angle2 = begin_angle[2] + 2*PI * (y/72.0)  * s1.getTCurve(count()-(s1.getTc()*1000)) ;
           controller()->motorPool()[2].setTargetPos(angle2);
       }
    if(((2*s1.getTc()*1000) < count()) && (count() <= (3*s1.getTc()*1000))){
           if(count() == time*2+1){
               begin_angle[0] = controller()->motorPool()[0].actualPos();
           }
           angle0 = begin_angle[0] - 2*PI*(x/72.0)*s1.getTCurve(count()-2*s1.getTc()*1000);
           controller()->motorPool()[0].setTargetPos(angle0);
       }
       if(((3*s1.getTc()*1000) < count()) && (count() <= (4*s1.getTc()*1000))){
           if(count() == time*3+1){
               begin_angle[2] = controller()->motorPool()[2].actualPos();

           }
           angle2 = begin_angle[2] - 2*PI*(y/72.0)*s1.getTCurve(count()-3*s1.getTc()*1000);
           controller()->motorPool()[2].setTargetPos(angle2);
       }

 //   controller()->motorPool()[0].setTargetPos(angle0);
 //   controller()->motorPool()[1].setTargetPos(angle1);
 //   controller()->motorPool()[2].setTargetPos(angle2);

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualCur() << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[0].actualToq() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[1].actualPos() <<"\t";
    lout() << controller()->motorPool()[2].actualPos() <<std::endl;
//    return 5000-count(); //运行时间为5秒
    return 4*s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto Rect::collectNrt()->void {}
Rect::Rect(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"rect\">"
       "	<GroupParam>"
       "	<Param name=\"x\" default=\"1\" abbreviation=\"x\"/>"
       "	<Param name=\"y\" default=\"1\" abbreviation=\"z\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
Rect::~Rect() = default;  //析构函数


//---------------------------------------------------------------------------------------------------模块控制-----------------------------------------------

//---------------------------------------------------右
//d
auto d::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto d::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto d::collectNrt()->void {}
d::d(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"d\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"30\" abbreviation=\"l\"/>"

//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
d::~d() = default;  //析构函数


//dd
auto dd::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto dd::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,4); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto dd::collectNrt()->void {}
dd::dd(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dd\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"80\" abbreviation=\"l\"/>"

//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
dd::~dd() = default;  //析构函数

//ddd
auto ddd::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ddd::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto ddd::collectNrt()->void {}
ddd::ddd(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ddd\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"150\" abbreviation=\"l\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
ddd::~ddd() = default;  //析构函数

//dddd
auto dddd::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto dddd::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1,0.8); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto dddd::collectNrt()->void {}
dddd::dddd(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dddd\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"300\" abbreviation=\"l\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
dddd::~dddd() = default;  //析构函数

//---------------------------------------------------左
//a
auto a::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto a::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto a::collectNrt()->void {}
a::a(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"a\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-30\" abbreviation=\"l\"/>"

//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
a::~a() = default;  //析构函数


//aa
auto aa::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto aa::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,4); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto aa::collectNrt()->void {}
aa::aa(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"aa\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-80\" abbreviation=\"l\"/>"

//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
aa::~aa() = default;  //析构函数

//aaa
auto aaa::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto aaa::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto aaa::collectNrt()->void {}
aaa::aaa(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"aaa\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-150\" abbreviation=\"l\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
aaa::~aaa() = default;  //析构函数

//aaaa
auto aaaa::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto aaaa::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1,0.8); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto aaaa::collectNrt()->void {}
aaaa::aaaa(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"aaaa\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-300\" abbreviation=\"l\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
aaaa::~aaaa() = default;  //析构函数

//---------------------------------------------------------上
//w
auto w::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto w::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto w::collectNrt()->void {}
w::w(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"w\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"30\" abbreviation=\"l\"/>"

       "	</GroupParam>"
       "</Command>");

}
w::~w() = default;  //析构函数

//ww
auto ww::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ww::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,4); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto ww::collectNrt()->void {}
ww::ww(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ww\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"80\" abbreviation=\"l\"/>"

       "	</GroupParam>"
       "</Command>");

}
ww::~ww() = default;  //析构函数

//www
auto www::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto www::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto www::collectNrt()->void {}
www::www(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"www\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"150\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
www::~www() = default;  //析构函数


//wwww
auto wwww::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto wwww::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1,0.8); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto wwww::collectNrt()->void {}
wwww::wwww(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"wwww\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"300\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
wwww::~wwww() = default;  //析构函数

//---------------------------------------------------------下
//s
auto s::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto s::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto s::collectNrt()->void {}
s::s(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"s\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-30\" abbreviation=\"l\"/>"

       "	</GroupParam>"
       "</Command>");

}
s::~s() = default;  //析构函数

//ss
auto ss::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ss::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,4); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto ss::collectNrt()->void {}
ss::ss(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ss\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-80\" abbreviation=\"l\"/>"

       "	</GroupParam>"
       "</Command>");

}
ss::~ss() = default;  //析构函数

//sss
auto sss::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto sss::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto sss::collectNrt()->void {}
sss::sss(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"sss\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-150\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
sss::~sss() = default;  //析构函数


//ssss
auto ssss::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ssss::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1,0.8); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto ssss::collectNrt()->void {}
ssss::ssss(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ssss\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-300\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
ssss::~ssss() = default;  //析构函数

//--------------------------------------------------------------上-------------------------
//r
auto r::prepareNrt()->void
{
    z_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto r::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[2].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[2].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto r::collectNrt()->void {}
r::r(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"r\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-10\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
r::~r() = default;  //析构函数

//rr
auto rr::prepareNrt()->void
{
    z_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto rr::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[2].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[2].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto rr::collectNrt()->void {}
rr::rr(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"rr\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"-150\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
rr::~rr() = default;  //析构函数

//--------------------------------------------------------------下-------------------------
//f
auto f::prepareNrt()->void
{
    z_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto f::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[2].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(4,6); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[2].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto f::collectNrt()->void {}
f::f(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"f\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"10\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
f::~f() = default;  //析构函数

//ff
auto ff::prepareNrt()->void
{
    z_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ff::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[2].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(2,2); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[2].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto ff::collectNrt()->void {}
ff::ff(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ff\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"150\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
ff::~ff() = default;  //析构函数
//----------------------------------------------------------------------------------------------------结束---------------------------------------------------------------


//xx
auto xx::prepareNrt()->void
{
    x_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto xx::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();

    }


//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * x_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[0].setTargetPos(angle0);

    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto xx::collectNrt()->void {}
xx::xx(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"xx\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"0\" abbreviation=\"l\"/>"
//     "	<Param name=\"coooo\" default=\"1\" abbreviation=\"a\"/>"
       "	</GroupParam>"
       "</Command>");

}
xx::~xx() = default;  //析构函数




//xx
auto yy::prepareNrt()->void
{
    y_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto yy::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[1].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * y_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[1].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto yy::collectNrt()->void {}
yy::yy(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"yy\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"0\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
yy::~yy() = default;  //析构函数



//xx
auto zz::prepareNrt()->void
{
    z_ = doubleParam("l");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto zz::executeRT()->int //进入实时线程
{
    static double begin_angle[1];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[2].actualPos();

    }

//    std::int32_t digits;
//    this->ecController()->motorPool()[0].readPdo(0x60fd, 0x00, digits);
//    if(count()%200==0) mout() << std::hex << digits << std::endl;

//  梯形曲线
    //mout()函数输出在终端上
    //lout()函数记录在文本中
    TCurve s1(1.5,1.5); //s1(a,v)
    s1.getCurveParam();
    double angle0 = begin_angle[0] + PI * 2 * z_/72.0  * s1.getTCurve(count()) ;//count()为循环次数计数，每1ms计数一次

    controller()->motorPool()[2].setTargetPos(angle0);


    return s1.getTc() * 1000-count(); //运行时间为T型曲线的周期
}

auto zz::collectNrt()->void {}
zz::zz(const std::string &name) //构造函数
{
    aris::core::fromXmlString(command(),
       "<Command name=\"zz\">"
       "	<GroupParam>"
       "	<Param name=\"l\" default=\"0\" abbreviation=\"l\"/>"
       "	</GroupParam>"
       "</Command>");

}
zz::~zz() = default;  //析构函数






//  速度模式
auto VelDrive::prepareNrt()->void{

    cef_ = doubleParam("coefficient");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto VelDrive::executeRT()->int{
    static double begin_vel[3];

    if (count()==1)
    {
        begin_vel[0] = controller()->motorPool()[0].actualVel();
        this->master()->logFileRawName("TestVel");
    }
    double vel0= begin_vel[0]+cef_*5.0*(1-std::cos(2*PI*count()/2000.0))/2;
    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    //log//
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[0].actualVel() <<std::endl;
    controller()->motorPool()[0].setTargetVel(vel0);
    return 20000-count();
}

auto VelDrive::collectNrt()->void{}
VelDrive::VelDrive(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_vel\">"
        "	<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"k\"/>"
        "</Command>");
}
VelDrive::~VelDrive() = default;  //析构函数




// 单关节正弦往复轨迹 //
struct MoveJSParam
{
    double j1;
    double time;
    uint32_t timenum;
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motorPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto &option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJS::executeRT()->int
{

    auto &param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //


    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motorPool()[0].actualPos();
            step_pjs = controller()->motorPool()[0].actualPos();
            this->master()->logFileRawName("moveJS");//建立记录数据的文件夹
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        controller()->motorPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motorPool()[0].actualPos();
            step_pjs = controller()->motorPool()[0].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        controller()->motorPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motorPool()[0].actualPos();
            step_pjs = controller()->motorPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        controller()->motorPool().at(0).setTargetPos(step_pjs);
    }

    // 打印 //
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }

    // log //

    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[0].actualVel() <<std::endl;

    return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJS\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}


auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>{
    std::unique_ptr<aris::control::Master> master(new aris::control::EthercatMaster);

    for (aris::Size i = 0; i < 3 ; ++i){
        int phy_id[3]={0,1,2};


        std::string xml_str =
            "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            ">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1600\" is_tx=\"false\">"
//            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
//            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
//            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"target_vel\" index=\"0x60B8\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"target_vel\" index=\"0x60FE\" subindex=\"0x01\" size=\"32\"/>"
//            "				<PdoEntry name=\"target_vel\" index=\"0x60FE\" subindex=\"0x02\" size=\"32\"/>"
                                                                     "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
                                                                     "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
                                                         //            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
                                                         //            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
                                                                     "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                                                                     "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"

            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1a00\" is_tx=\"true\">"
//"                <PdoEntry name=\"object\" index=\"0x603f\" subindex=\"0x00\" size=\"16\"/>"
//"                <PdoEntry name=\"object\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
//"                <PdoEntry name=\"object\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
//"                <PdoEntry name=\"object\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60bb\" subindex=\"0x00\" size=\"32\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60bc\" subindex=\"0x00\" size=\"32\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60bd\" subindex=\"0x00\" size=\"32\"/>"
//"                <PdoEntry name=\"object\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"


                                                                     "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                                                                     "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
                                                                     "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                                                                     "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
                                                                     "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
                                                                     "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatSlave>";

//        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
//        "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
//         "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
//         "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
//         "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
//         "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"

       auto& s = master->slavePool().add<aris::control::EthercatSlave>();
       aris::core::fromXmlString(s, xml_str);
    #ifdef WIN32
            dynamic_cast<aris::control::EthercatSlave&>(master->slavePool().back()).setVirtual(true);
    #endif
    #ifndef WIN32
            dynamic_cast<aris::control::EthercatSlave&>(master->slavePool().back()).scanInfoForCurrentSlave();
            //dynamic_cast<aris::control::EthercatSlave&>(master->slavePool().back()).scanPdoForCurrentSlave();  //扫描电机pdo
    #endif

       s.setSync0ShiftNs(900000);
       s.setDcAssignActivate(0x300);

    }
    return master;

}
auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::Controller);

    for (aris::Size i = 0; i < 3 ; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[3]
        {
            0,0,0
        };
#else
        double pos_offset[3]
        {
            0
        };
#endif
        double pos_factor[3] //偏置系数
        {
            2000/PI,2000/PI,2000/PI
        };
        double max_pos[3] //最大位置
        {
            500*PI,500*PI,500*PI
        };
        double min_pos[3] //最小位置
        {
            -500*PI,-500*PI,-500*PI
        };
        double max_vel[3]  //最大速度
        {
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[3]  //最大加速度
        {
            3000,  3000,  3000
        };

        //zero_err
        std::string xml_str =
            "<EthercatMotor min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\" slave=\""+std::to_string(i) + "\">"
            "</EthercatMotor>";

       auto& s = controller->motorPool().add<aris::control::EthercatMotor>();
               aris::core::fromXmlString(s, xml_str);
       s.setEnableWaitingCount(100);

    };
    return controller;
}
auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //自己写的命令

    plan_root->planPool().add<VelDrive>();
    plan_root->planPool().add<TcurveDrive>();
    plan_root->planPool().add<TcurveDrive1>();
    plan_root->planPool().add<BackZero>();
    plan_root->planPool().add<Rect>();
    plan_root->planPool().add<MoveJS>();
    plan_root->planPool().add<xx>();
    plan_root->planPool().add<yy>();
    plan_root->planPool().add<zz>();
    plan_root->planPool().add<w>();
    plan_root->planPool().add<ww>();
    plan_root->planPool().add<www>();
    plan_root->planPool().add<wwww>();
    plan_root->planPool().add<s>();
    plan_root->planPool().add<ss>();
    plan_root->planPool().add<sss>();
    plan_root->planPool().add<ssss>();
    plan_root->planPool().add<d>();
    plan_root->planPool().add<dd>();
    plan_root->planPool().add<ddd>();
    plan_root->planPool().add<dddd>();
    plan_root->planPool().add<a>();
    plan_root->planPool().add<aa>();
    plan_root->planPool().add<aaa>();
    plan_root->planPool().add<aaaa>();
    plan_root->planPool().add<r>();
    plan_root->planPool().add<rr>();
    plan_root->planPool().add<f>();
    plan_root->planPool().add<ff>();


    return plan_root;
}















}







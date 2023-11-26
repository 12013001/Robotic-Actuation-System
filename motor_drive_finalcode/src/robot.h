#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{



    class VelDrive : public aris::core::CloneObject<VelDrive,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~VelDrive();
        explicit VelDrive(const std::string &name = "vel_drive");

     private:
        double cef_;
    };


    class TcurveDrive :public aris::core::CloneObject<TcurveDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~TcurveDrive();
        explicit TcurveDrive(const std::string &name = "motor_drive");

    private:
        double cef_;
        double aaa;
    };

    //t1
    class TcurveDrive1 :public aris::core::CloneObject<TcurveDrive1,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~TcurveDrive1();
        explicit TcurveDrive1(const std::string &name = "motor_drive_1");

    private:
        double x_;
        double y_;
        double z_;
    };

    class BackZero :public aris::core::CloneObject<BackZero,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~BackZero();
        explicit BackZero(const std::string &name = "motor_drive_2");

    private:
        double x_;
        double y_;
        double z_;
    };

    class Rect :public aris::core::CloneObject<Rect,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~Rect();
        explicit Rect(const std::string &name = "motor_drive_3");

    private:
        double x;
        double y;
    };

    class xx :public aris::core::CloneObject<xx,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~xx();
        explicit xx(const std::string &name = "xx");

    private:
       double x_;
       double y_;
       double z_;
};
    class yy :public aris::core::CloneObject<yy,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~yy();
        explicit yy(const std::string &name = "yy");

    private:
       double x_;
       double y_;
       double z_;
};
    class zz :public aris::core::CloneObject<zz,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~zz();
        explicit zz(const std::string &name = "zz");

    private:
       double x_;
       double y_;
       double z_;
};

//--------------------------------------------------------------------------------------------------
    class d :public aris::core::CloneObject<d,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~d();
        explicit d(const std::string &name = "d");

    private:
       double x_;
       double y_;
       double z_;
};
    class dd :public aris::core::CloneObject<dd,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~dd();
        explicit dd(const std::string &name = "dd");

    private:
       double x_;
       double y_;
       double z_;
};
    class ddd :public aris::core::CloneObject<ddd,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ddd();
        explicit ddd(const std::string &name = "ddd");

    private:
       double x_;
       double y_;
       double z_;
};
    class dddd :public aris::core::CloneObject<dddd,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~dddd();
        explicit dddd(const std::string &name = "dddd");

    private:
       double x_;
       double y_;
       double z_;
};
    class a :public aris::core::CloneObject<a,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~a();
        explicit a(const std::string &name = "a");

    private:
       double x_;
       double y_;
       double z_;
};
    class aa :public aris::core::CloneObject<aa,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~aa();
        explicit aa(const std::string &name = "aa");

    private:
       double x_;
       double y_;
       double z_;
};
    class aaa :public aris::core::CloneObject<aaa,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~aaa();
        explicit aaa(const std::string &name = "aaa");

    private:
       double x_;
       double y_;
       double z_;
};
    class aaaa :public aris::core::CloneObject<aaaa,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~aaaa();
        explicit aaaa(const std::string &name = "aaaa");

    private:
       double x_;
       double y_;
       double z_;
};
    class w :public aris::core::CloneObject<w,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~w();
        explicit w(const std::string &name = "w");

    private:
       double x_;
       double y_;
       double z_;
};
    class ww :public aris::core::CloneObject<ww,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ww();
        explicit ww(const std::string &name = "ww");

    private:
       double x_;
       double y_;
       double z_;
};
    class www :public aris::core::CloneObject<www,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~www();
        explicit www(const std::string &name = "www");

    private:
       double x_;
       double y_;
       double z_;
};
    class wwww :public aris::core::CloneObject<wwww,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~wwww();
        explicit wwww(const std::string &name = "wwww");

    private:
       double x_;
       double y_;
       double z_;
};
    class s :public aris::core::CloneObject<s,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~s();
        explicit s(const std::string &name = "s");

    private:
       double x_;
       double y_;
       double z_;
};
    class ss :public aris::core::CloneObject<ss,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ss();
        explicit ss(const std::string &name = "ss");

    private:
       double x_;
       double y_;
       double z_;
};
    class sss :public aris::core::CloneObject<sss,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~sss();
        explicit sss(const std::string &name = "sss");

    private:
       double x_;
       double y_;
       double z_;
};
    class ssss :public aris::core::CloneObject<ssss,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ssss();
        explicit ssss(const std::string &name = "ssss");

    private:
       double x_;
       double y_;
       double z_;
};
    class r :public aris::core::CloneObject<r,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~r();
        explicit r(const std::string &name = "r");

    private:
       double x_;
       double y_;
       double z_;
};
    class rr :public aris::core::CloneObject<rr,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~rr();
        explicit rr(const std::string &name = "rr");

    private:
       double x_;
       double y_;
       double z_;
};
    class f :public aris::core::CloneObject<f,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~f();
        explicit f(const std::string &name = "f");

    private:
       double x_;
       double y_;
       double z_;
};
    class ff :public aris::core::CloneObject<ff,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ff();
        explicit ff(const std::string &name = "ff");

    private:
       double x_;
       double y_;
       double z_;
};
//--------------------------------------------------------------------------------------------------------------------

    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
      {
      public:
          auto virtual prepareNrt()->void;
          auto virtual executeRT()->int;
          auto virtual collectNrt()->void;

          explicit MoveJS(const std::string &name = "MoveJS_plan");

      };

    auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>;
    auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>;
    auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif

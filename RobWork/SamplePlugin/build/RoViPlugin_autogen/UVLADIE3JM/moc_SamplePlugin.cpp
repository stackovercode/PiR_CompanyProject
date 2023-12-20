/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SamplePlugin_t {
    const uint offsetsAndSize[160];
    char stringdata0[1091];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 12), // "SamplePlugin"
QT_MOC_LITERAL(13, 10), // "btnPressed"
QT_MOC_LITERAL(24, 0), // ""
QT_MOC_LITERAL(25, 13), // "interpolation"
QT_MOC_LITERAL(39, 1), // "Q"
QT_MOC_LITERAL(41, 2), // "Q1"
QT_MOC_LITERAL(44, 2), // "Q2"
QT_MOC_LITERAL(47, 6), // "sample"
QT_MOC_LITERAL(54, 5), // "timer"
QT_MOC_LITERAL(60, 8), // "getImage"
QT_MOC_LITERAL(69, 11), // "get25DImage"
QT_MOC_LITERAL(81, 19), // "getCollectImageTest"
QT_MOC_LITERAL(101, 14), // "poseEstimation"
QT_MOC_LITERAL(116, 7), // "cv::Mat"
QT_MOC_LITERAL(124, 21), // "printProjectionMatrix"
QT_MOC_LITERAL(146, 11), // "std::string"
QT_MOC_LITERAL(158, 9), // "frameName"
QT_MOC_LITERAL(168, 8), // "cv::Mat&"
QT_MOC_LITERAL(177, 8), // "proj_mat"
QT_MOC_LITERAL(186, 7), // "cam_mat"
QT_MOC_LITERAL(194, 25), // "rw::models::WorkCell::Ptr"
QT_MOC_LITERAL(220, 2), // "wc"
QT_MOC_LITERAL(223, 12), // "moveCylinder"
QT_MOC_LITERAL(236, 5), // "index"
QT_MOC_LITERAL(242, 8), // "findPose"
QT_MOC_LITERAL(251, 26), // "std::pair<cv::Mat,cv::Mat>"
QT_MOC_LITERAL(278, 8), // "left_img"
QT_MOC_LITERAL(287, 9), // "right_img"
QT_MOC_LITERAL(297, 11), // "projLeftMat"
QT_MOC_LITERAL(309, 12), // "projRightMat"
QT_MOC_LITERAL(322, 12), // "locateBallV2"
QT_MOC_LITERAL(335, 30), // "std::pair<cv::Point2d,cv::Mat>"
QT_MOC_LITERAL(366, 16), // "undistortedImage"
QT_MOC_LITERAL(383, 3), // "P2P"
QT_MOC_LITERAL(387, 4), // "pose"
QT_MOC_LITERAL(392, 12), // "constant_vel"
QT_MOC_LITERAL(405, 1), // "t"
QT_MOC_LITERAL(407, 2), // "t0"
QT_MOC_LITERAL(410, 2), // "t1"
QT_MOC_LITERAL(413, 19), // "resetRobotAndObject"
QT_MOC_LITERAL(433, 72), // "std::pair<std::vector<rw::mat..."
QT_MOC_LITERAL(506, 36), // "std::vector<rw::math::Transfo..."
QT_MOC_LITERAL(543, 1), // "P"
QT_MOC_LITERAL(545, 18), // "std::vector<float>"
QT_MOC_LITERAL(564, 1), // "T"
QT_MOC_LITERAL(566, 29), // "rw::kinematics::MovableFrame*"
QT_MOC_LITERAL(596, 11), // "targetFrame"
QT_MOC_LITERAL(608, 29), // "rw::models::SerialDevice::Ptr"
QT_MOC_LITERAL(638, 9), // "robot_ur5"
QT_MOC_LITERAL(648, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(670, 5), // "state"
QT_MOC_LITERAL(676, 33), // "rw::proximity::CollisionDetec..."
QT_MOC_LITERAL(710, 8), // "detector"
QT_MOC_LITERAL(719, 25), // "findCollisionFreeSolution"
QT_MOC_LITERAL(745, 11), // "rw::math::Q"
QT_MOC_LITERAL(757, 24), // "std::vector<rw::math::Q>"
QT_MOC_LITERAL(782, 9), // "solutions"
QT_MOC_LITERAL(792, 12), // "prevSolution"
QT_MOC_LITERAL(805, 20), // "stateChangedListener"
QT_MOC_LITERAL(826, 15), // "checkCollisions"
QT_MOC_LITERAL(842, 11), // "Device::Ptr"
QT_MOC_LITERAL(854, 6), // "device"
QT_MOC_LITERAL(861, 5), // "State"
QT_MOC_LITERAL(867, 17), // "CollisionDetector"
QT_MOC_LITERAL(885, 1), // "q"
QT_MOC_LITERAL(887, 20), // "createPathRRTConnect"
QT_MOC_LITERAL(908, 4), // "from"
QT_MOC_LITERAL(913, 2), // "to"
QT_MOC_LITERAL(916, 6), // "extend"
QT_MOC_LITERAL(923, 7), // "maxTime"
QT_MOC_LITERAL(931, 16), // "constantVelocity"
QT_MOC_LITERAL(948, 20), // "convertToTransform3D"
QT_MOC_LITERAL(969, 29), // "rw::math::Transform3D<double>"
QT_MOC_LITERAL(999, 15), // "Eigen::Vector3d"
QT_MOC_LITERAL(1015, 11), // "translation"
QT_MOC_LITERAL(1027, 18), // "Eigen::Quaterniond"
QT_MOC_LITERAL(1046, 8), // "rotation"
QT_MOC_LITERAL(1055, 18), // "findConfigurations"
QT_MOC_LITERAL(1074, 8), // "nameGoal"
QT_MOC_LITERAL(1083, 7) // "nameTcp"

    },
    "SamplePlugin\0btnPressed\0\0interpolation\0"
    "Q\0Q1\0Q2\0sample\0timer\0getImage\0get25DImage\0"
    "getCollectImageTest\0poseEstimation\0"
    "cv::Mat\0printProjectionMatrix\0std::string\0"
    "frameName\0cv::Mat&\0proj_mat\0cam_mat\0"
    "rw::models::WorkCell::Ptr\0wc\0moveCylinder\0"
    "index\0findPose\0std::pair<cv::Mat,cv::Mat>\0"
    "left_img\0right_img\0projLeftMat\0"
    "projRightMat\0locateBallV2\0"
    "std::pair<cv::Point2d,cv::Mat>\0"
    "undistortedImage\0P2P\0pose\0constant_vel\0"
    "t\0t0\0t1\0resetRobotAndObject\0"
    "std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>"
    ">>\0"
    "std::vector<rw::math::Transform3D<>>\0"
    "P\0std::vector<float>\0T\0"
    "rw::kinematics::MovableFrame*\0targetFrame\0"
    "rw::models::SerialDevice::Ptr\0robot_ur5\0"
    "rw::kinematics::State\0state\0"
    "rw::proximity::CollisionDetector&\0"
    "detector\0findCollisionFreeSolution\0"
    "rw::math::Q\0std::vector<rw::math::Q>\0"
    "solutions\0prevSolution\0stateChangedListener\0"
    "checkCollisions\0Device::Ptr\0device\0"
    "State\0CollisionDetector\0q\0"
    "createPathRRTConnect\0from\0to\0extend\0"
    "maxTime\0constantVelocity\0convertToTransform3D\0"
    "rw::math::Transform3D<double>\0"
    "Eigen::Vector3d\0translation\0"
    "Eigen::Quaterniond\0rotation\0"
    "findConfigurations\0nameGoal\0nameTcp"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  164,    2, 0x08,    1 /* Private */,
       3,    3,  165,    2, 0x08,    2 /* Private */,
       8,    0,  172,    2, 0x08,    6 /* Private */,
       9,    0,  173,    2, 0x08,    7 /* Private */,
      10,    0,  174,    2, 0x08,    8 /* Private */,
      11,    0,  175,    2, 0x08,    9 /* Private */,
      12,    0,  176,    2, 0x08,   10 /* Private */,
      14,    3,  177,    2, 0x08,   11 /* Private */,
      14,    4,  184,    2, 0x08,   15 /* Private */,
      22,    1,  193,    2, 0x08,   20 /* Private */,
      24,    4,  196,    2, 0x08,   22 /* Private */,
      30,    1,  205,    2, 0x08,   27 /* Private */,
      33,    1,  208,    2, 0x08,   29 /* Private */,
      35,    3,  211,    2, 0x08,   31 /* Private */,
      39,    0,  218,    2, 0x08,   35 /* Private */,
      33,    7,  219,    2, 0x08,   36 /* Private */,
      53,    4,  234,    2, 0x08,   44 /* Private */,
      53,    5,  243,    2, 0x08,   49 /* Private */,
      58,    1,  254,    2, 0x08,   55 /* Private */,
      59,    4,  257,    2, 0x08,   57 /* Private */,
      65,    4,  266,    2, 0x08,   62 /* Private */,
      14,    1,  275,    2, 0x08,   67 /* Private */,
      70,    3,  278,    2, 0x08,   69 /* Private */,
      71,    2,  285,    2, 0x08,   73 /* Private */,
      77,    5,  290,    2, 0x08,   76 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4, 0x80000000 | 4, QMetaType::Int,    5,    6,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 13,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 17, 0x80000000 | 17,   16,   18,   19,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 20, 0x80000000 | 17, 0x80000000 | 17,   16,   21,   18,   19,
    QMetaType::Void, QMetaType::Int,   23,
    0x80000000 | 25, 0x80000000 | 13, 0x80000000 | 13, 0x80000000 | 13, 0x80000000 | 13,   26,   27,   28,   29,
    0x80000000 | 31, 0x80000000 | 13,   32,
    QMetaType::Void, 0x80000000 | 13,   34,
    QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   36,   37,   38,
    QMetaType::Void,
    0x80000000 | 40, 0x80000000 | 41, 0x80000000 | 43, 0x80000000 | 45, 0x80000000 | 47, 0x80000000 | 20, 0x80000000 | 49, 0x80000000 | 51,   42,   44,   46,   48,   21,   50,   52,
    0x80000000 | 54, 0x80000000 | 47, 0x80000000 | 49, 0x80000000 | 51, 0x80000000 | 55,   48,   50,   52,   56,
    0x80000000 | 54, 0x80000000 | 47, 0x80000000 | 49, 0x80000000 | 51, 0x80000000 | 55, 0x80000000 | 54,   48,   50,   52,   56,   57,
    QMetaType::Void, 0x80000000 | 49,   50,
    QMetaType::Bool, 0x80000000 | 60, 0x80000000 | 62, 0x80000000 | 63, 0x80000000 | 4,   61,   50,   52,   64,
    QMetaType::Void, 0x80000000 | 4, 0x80000000 | 4, QMetaType::Double, QMetaType::Double,   66,   67,   68,   69,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   36,   37,   38,
    0x80000000 | 72, 0x80000000 | 73, 0x80000000 | 75,   74,   76,
    0x80000000 | 55, 0x80000000 | 15, 0x80000000 | 15, 0x80000000 | 47, 0x80000000 | 20, 0x80000000 | 49,   78,   79,   48,   21,   50,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SamplePlugin *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->interpolation((*reinterpret_cast< std::add_pointer_t<Q>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[3]))); break;
        case 2: _t->timer(); break;
        case 3: _t->getImage(); break;
        case 4: _t->get25DImage(); break;
        case 5: _t->getCollectImageTest(); break;
        case 6: { cv::Mat _r = _t->poseEstimation();
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 7: _t->printProjectionMatrix((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[3]))); break;
        case 8: _t->printProjectionMatrix((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<rw::models::WorkCell::Ptr>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<cv::Mat&>>(_a[4]))); break;
        case 9: _t->moveCylinder((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 10: { std::pair<cv::Mat,cv::Mat> _r = _t->findPose((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::pair<cv::Mat,cv::Mat>*>(_a[0]) = std::move(_r); }  break;
        case 11: { std::pair<cv::Point2d,cv::Mat> _r = _t->locateBallV2((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::pair<cv::Point2d,cv::Mat>*>(_a[0]) = std::move(_r); }  break;
        case 12: _t->P2P((*reinterpret_cast< std::add_pointer_t<cv::Mat>>(_a[1]))); break;
        case 13: { double _r = _t->constant_vel((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->resetRobotAndObject(); break;
        case 15: { std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> _r = _t->P2P((*reinterpret_cast< std::add_pointer_t<std::vector<rw::math::Transform3D<>>>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::vector<float>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::kinematics::MovableFrame*>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<rw::models::WorkCell::Ptr>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[6])),(*reinterpret_cast< std::add_pointer_t<rw::proximity::CollisionDetector&>>(_a[7])));
            if (_a[0]) *reinterpret_cast< std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>>*>(_a[0]) = std::move(_r); }  break;
        case 16: { rw::math::Q _r = _t->findCollisionFreeSolution((*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::proximity::CollisionDetector&>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<std::vector<rw::math::Q>>>(_a[4])));
            if (_a[0]) *reinterpret_cast< rw::math::Q*>(_a[0]) = std::move(_r); }  break;
        case 17: { rw::math::Q _r = _t->findCollisionFreeSolution((*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::proximity::CollisionDetector&>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<std::vector<rw::math::Q>>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<rw::math::Q>>(_a[5])));
            if (_a[0]) *reinterpret_cast< rw::math::Q*>(_a[0]) = std::move(_r); }  break;
        case 18: _t->stateChangedListener((*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[1]))); break;
        case 19: { bool _r = _t->checkCollisions((*reinterpret_cast< std::add_pointer_t<Device::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<State>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<CollisionDetector>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 20: _t->createPathRRTConnect((*reinterpret_cast< std::add_pointer_t<Q>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<Q>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[4]))); break;
        case 21: _t->printProjectionMatrix((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1]))); break;
        case 22: { double _r = _t->constantVelocity((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 23: { rw::math::Transform3D<double> _r = _t->convertToTransform3D((*reinterpret_cast< std::add_pointer_t<Eigen::Vector3d>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<Eigen::Quaterniond>>(_a[2])));
            if (_a[0]) *reinterpret_cast< rw::math::Transform3D<double>*>(_a[0]) = std::move(_r); }  break;
        case 24: { std::vector<rw::math::Q> _r = _t->findConfigurations((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<std::string>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<rw::models::SerialDevice::Ptr>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<rw::models::WorkCell::Ptr>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<rw::kinematics::State>>(_a[5])));
            if (_a[0]) *reinterpret_cast< std::vector<rw::math::Q>*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject SamplePlugin::staticMetaObject = { {
    QMetaObject::SuperData::link<rws::RobWorkStudioPlugin::staticMetaObject>(),
    qt_meta_stringdata_SamplePlugin.offsetsAndSize,
    qt_meta_data_SamplePlugin,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SamplePlugin_t
, QtPrivate::TypeAndForceComplete<SamplePlugin, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::WorkCell::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<std::pair<cv::Mat,cv::Mat>, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<std::pair<cv::Point2d,cv::Mat>, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<cv::Mat, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>> >, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<rw::math::Transform3D<>>, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<float>, std::false_type>, QtPrivate::TypeAndForceComplete<rw::kinematics::MovableFrame *, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::WorkCell::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::kinematics::State, std::false_type>, QtPrivate::TypeAndForceComplete<rw::proximity::CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<rw::math::Q, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::kinematics::State, std::false_type>, QtPrivate::TypeAndForceComplete<rw::proximity::CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<rw::math::Q>, std::false_type>, QtPrivate::TypeAndForceComplete<rw::math::Q, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::kinematics::State, std::false_type>, QtPrivate::TypeAndForceComplete<rw::proximity::CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<rw::math::Q>, std::false_type>, QtPrivate::TypeAndForceComplete<rw::math::Q, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const rw::kinematics::State &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<Device::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<const State &, std::false_type>, QtPrivate::TypeAndForceComplete<const CollisionDetector &, std::false_type>, QtPrivate::TypeAndForceComplete<const Q &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<Q, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<rw::math::Transform3D<double>, std::false_type>, QtPrivate::TypeAndForceComplete<const Eigen::Vector3d, std::false_type>, QtPrivate::TypeAndForceComplete<const Eigen::Quaterniond, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<rw::math::Q>, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<const std::string, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::SerialDevice::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::models::WorkCell::Ptr, std::false_type>, QtPrivate::TypeAndForceComplete<rw::kinematics::State, std::false_type>


>,
    nullptr
} };


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(this);
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 25;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION
static constexpr unsigned char qt_pluginMetaData_SamplePlugin[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', '!',
    // metadata version, Qt version, architectural requirements
    0, QT_VERSION_MAJOR, QT_VERSION_MINOR, qPluginArchRequirements(),
    0xbf, 
    // "IID"
    0x02,  0x78,  0x2a,  'd',  'k',  '.',  's',  'd', 
    'u',  '.',  'm',  'i',  'p',  '.',  'R',  'o', 
    'b',  'w',  'o',  'r',  'k',  '.',  'R',  'o', 
    'b',  'W',  'o',  'r',  'k',  'S',  't',  'u', 
    'd',  'i',  'o',  'P',  'l',  'u',  'g',  'i', 
    'n',  '/',  '0',  '.',  '1', 
    // "className"
    0x03,  0x6c,  'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n', 
    // "MetaData"
    0x04,  0xa3,  0x6c,  'd',  'e',  'p',  'e',  'n', 
    'd',  'e',  'n',  'c',  'i',  'e',  's',  0x80, 
    0x64,  'n',  'a',  'm',  'e',  0x6b,  'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x67,  'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x65,  '1',  '.',  '0',  '.',  '0', 
    0xff, 
};
QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_WARNING_POP
QT_END_MOC_NAMESPACE

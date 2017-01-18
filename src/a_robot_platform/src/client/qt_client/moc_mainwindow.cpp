/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[14];
    char stringdata0[263];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 9), // "ShowLaser"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 14), // "ShowUltrasonic"
QT_MOC_LITERAL(4, 37, 19), // "on_ultraAll_clicked"
QT_MOC_LITERAL(5, 57, 29), // "on_pBtn_start2connect_clicked"
QT_MOC_LITERAL(6, 87, 7), // "checked"
QT_MOC_LITERAL(7, 95, 32), // "on_pBtn_key_control_open_clicked"
QT_MOC_LITERAL(8, 128, 25), // "on_lEdit_ip_returnPressed"
QT_MOC_LITERAL(9, 154, 27), // "on_lEdit_port_returnPressed"
QT_MOC_LITERAL(10, 182, 12), // "xTimerUpdate"
QT_MOC_LITERAL(11, 195, 14), // "cmdTimerUpdate"
QT_MOC_LITERAL(12, 210, 24), // "on_pBtn_open_map_clicked"
QT_MOC_LITERAL(13, 235, 27) // "on_pBtn_open_submap_clicked"

    },
    "MainWindow\0ShowLaser\0\0ShowUltrasonic\0"
    "on_ultraAll_clicked\0on_pBtn_start2connect_clicked\0"
    "checked\0on_pBtn_key_control_open_clicked\0"
    "on_lEdit_ip_returnPressed\0"
    "on_lEdit_port_returnPressed\0xTimerUpdate\0"
    "cmdTimerUpdate\0on_pBtn_open_map_clicked\0"
    "on_pBtn_open_submap_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x08 /* Private */,
       3,    0,   70,    2, 0x08 /* Private */,
       4,    0,   71,    2, 0x08 /* Private */,
       5,    1,   72,    2, 0x08 /* Private */,
       7,    1,   75,    2, 0x08 /* Private */,
       8,    0,   78,    2, 0x08 /* Private */,
       9,    0,   79,    2, 0x08 /* Private */,
      10,    0,   80,    2, 0x08 /* Private */,
      11,    0,   81,    2, 0x08 /* Private */,
      12,    0,   82,    2, 0x08 /* Private */,
      13,    0,   83,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ShowLaser(); break;
        case 1: _t->ShowUltrasonic(); break;
        case 2: _t->on_ultraAll_clicked(); break;
        case 3: _t->on_pBtn_start2connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_pBtn_key_control_open_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_lEdit_ip_returnPressed(); break;
        case 6: _t->on_lEdit_port_returnPressed(); break;
        case 7: _t->xTimerUpdate(); break;
        case 8: _t->cmdTimerUpdate(); break;
        case 9: _t->on_pBtn_open_map_clicked(); break;
        case 10: _t->on_pBtn_open_submap_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

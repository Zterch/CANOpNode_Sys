/****************************************************************************
** Meta object code from reading C++ file 'datarecorder.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "datarecorder.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'datarecorder.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DataRecorder_t {
    QByteArrayData data[12];
    char stringdata0[120];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DataRecorder_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DataRecorder_t qt_meta_stringdata_DataRecorder = {
    {
QT_MOC_LITERAL(0, 0, 12), // "DataRecorder"
QT_MOC_LITERAL(1, 13, 16), // "recordingStarted"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 8), // "filePath"
QT_MOC_LITERAL(4, 40, 16), // "recordingStopped"
QT_MOC_LITERAL(5, 57, 12), // "dataRecorded"
QT_MOC_LITERAL(6, 70, 5), // "count"
QT_MOC_LITERAL(7, 76, 5), // "error"
QT_MOC_LITERAL(8, 82, 7), // "message"
QT_MOC_LITERAL(9, 90, 13), // "onDataUpdated"
QT_MOC_LITERAL(10, 104, 10), // "SensorData"
QT_MOC_LITERAL(11, 115, 4) // "data"

    },
    "DataRecorder\0recordingStarted\0\0filePath\0"
    "recordingStopped\0dataRecorded\0count\0"
    "error\0message\0onDataUpdated\0SensorData\0"
    "data"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DataRecorder[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       4,    0,   42,    2, 0x06 /* Public */,
       5,    1,   43,    2, 0x06 /* Public */,
       7,    1,   46,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,   49,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, QMetaType::QString,    8,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void DataRecorder::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DataRecorder *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->recordingStarted((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->recordingStopped(); break;
        case 2: _t->dataRecorded((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->onDataUpdated((*reinterpret_cast< const SensorData(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DataRecorder::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DataRecorder::recordingStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DataRecorder::recordingStopped)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DataRecorder::dataRecorded)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DataRecorder::error)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject DataRecorder::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_DataRecorder.data,
    qt_meta_data_DataRecorder,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DataRecorder::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DataRecorder::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DataRecorder.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int DataRecorder::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void DataRecorder::recordingStarted(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DataRecorder::recordingStopped()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void DataRecorder::dataRecorded(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DataRecorder::error(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

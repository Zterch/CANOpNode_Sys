/****************************************************************************
** Meta object code from reading C++ file 'shmdatacollector.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../shmdatacollector.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'shmdatacollector.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ShmDataCollector_t {
    QByteArrayData data[14];
    char stringdata0[136];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ShmDataCollector_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ShmDataCollector_t qt_meta_stringdata_ShmDataCollector = {
    {
QT_MOC_LITERAL(0, 0, 16), // "ShmDataCollector"
QT_MOC_LITERAL(1, 17, 7), // "started"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 7), // "stopped"
QT_MOC_LITERAL(4, 34, 9), // "connected"
QT_MOC_LITERAL(5, 44, 12), // "disconnected"
QT_MOC_LITERAL(6, 57, 5), // "error"
QT_MOC_LITERAL(7, 63, 7), // "message"
QT_MOC_LITERAL(8, 71, 12), // "dataReceived"
QT_MOC_LITERAL(9, 84, 8), // "uint32_t"
QT_MOC_LITERAL(10, 93, 8), // "sequence"
QT_MOC_LITERAL(11, 102, 8), // "uint64_t"
QT_MOC_LITERAL(12, 111, 12), // "timestamp_us"
QT_MOC_LITERAL(13, 124, 11) // "collectData"

    },
    "ShmDataCollector\0started\0\0stopped\0"
    "connected\0disconnected\0error\0message\0"
    "dataReceived\0uint32_t\0sequence\0uint64_t\0"
    "timestamp_us\0collectData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ShmDataCollector[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    0,   51,    2, 0x06 /* Public */,
       5,    0,   52,    2, 0x06 /* Public */,
       6,    1,   53,    2, 0x06 /* Public */,
       8,    2,   56,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    0,   61,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, 0x80000000 | 9, 0x80000000 | 11,   10,   12,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void ShmDataCollector::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ShmDataCollector *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->started(); break;
        case 1: _t->stopped(); break;
        case 2: _t->connected(); break;
        case 3: _t->disconnected(); break;
        case 4: _t->error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->dataReceived((*reinterpret_cast< uint32_t(*)>(_a[1])),(*reinterpret_cast< uint64_t(*)>(_a[2]))); break;
        case 6: _t->collectData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ShmDataCollector::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::started)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ShmDataCollector::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::stopped)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ShmDataCollector::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::connected)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ShmDataCollector::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::disconnected)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ShmDataCollector::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::error)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (ShmDataCollector::*)(uint32_t , uint64_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ShmDataCollector::dataReceived)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ShmDataCollector::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_ShmDataCollector.data,
    qt_meta_data_ShmDataCollector,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ShmDataCollector::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ShmDataCollector::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ShmDataCollector.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ShmDataCollector::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ShmDataCollector::started()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void ShmDataCollector::stopped()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void ShmDataCollector::connected()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void ShmDataCollector::disconnected()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void ShmDataCollector::error(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ShmDataCollector::dataReceived(uint32_t _t1, uint64_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

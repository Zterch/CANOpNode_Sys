#include "chartwidget.h"
#include <QPainter>
#include <QDebug>
#include <QElapsedTimer>

ChartWidget::ChartWidget(DataModel *model, QWidget *parent) : QWidget(parent), m_dataModel(model)
{
    setMinimumSize(800, 600);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    // 设置绘图区域
    m_plotRect = QRect(m_marginLeft, m_marginTop, 
                       width() - m_marginLeft - m_marginRight,
                       height() - m_marginTop - m_marginBottom);
    
    m_startTime = QDateTime::currentDateTime();
    
    // 设置背景色
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setPalette(pal);
    
    // 连接DataModel的数据更新信号（关键修复：改为事件驱动）
    if (m_dataModel) {
        connect(m_dataModel, &DataModel::dataUpdated, this, &ChartWidget::onDataUpdated);
    }
    
    // 初始化图表更新定时器（用于强制刷新，20Hz）
    m_updateTimer = new QTimer(this);
    m_updateTimer->setTimerType(Qt::PreciseTimer);
    m_updateTimer->setInterval(50);  // 50ms = 20Hz
    connect(m_updateTimer, &QTimer::timeout, this, &ChartWidget::onUpdateTimer);
    m_updateTimer->start();
    
    // 初始化统计定时器
    m_statsTimer = new QElapsedTimer();
    m_statsTimer->start();
    m_frameCount = 0;
}

ChartWidget::~ChartWidget()
{
    if (m_updateTimer) {
        m_updateTimer->stop();
    }
}

void ChartWidget::setUpdateRate(int hz)
{
    if (hz > 0 && m_updateTimer) {
        int intervalMs = 1000 / hz;
        m_updateTimer->setInterval(intervalMs);
    }
}

void ChartWidget::addDataPoint(const SensorData &data)
{
    // 添加数据到缓冲区
    m_dataBuffer.append(data);
    
    // 限制缓冲区大小
    if (m_dataBuffer.size() > m_maxPoints) {
        m_dataBuffer.removeFirst();
    }
    
    // 更新Y轴范围（不重绘，由定时器统一刷新）
    calculateDataRange(m_yMinLeft, m_yMaxLeft, 0);  // 左侧Y轴（压力/绳长）
    calculateDataRange(m_yMinRight, m_yMaxRight, 2); // 右侧Y轴（电流/电压）
}

void ChartWidget::clearData()
{
    m_dataBuffer.clear();
    m_startTime = QDateTime::currentDateTime();
    update();
}

void ChartWidget::setDisplayType(int type)
{
    m_displayType = type;
    update();
}

void ChartWidget::loadFromHistory(const QVector<SensorData> &history)
{
    clearData();
    
    if (history.isEmpty()) {
        return;
    }
    
    // 设置起始时间
    m_startTime = history.first().timestamp;
    
    // 添加所有数据点
    for (const auto &data : history) {
        m_dataBuffer.append(data);
    }
    
    // 更新Y轴范围
    calculateDataRange(m_yMinLeft, m_yMaxLeft, 0);
    calculateDataRange(m_yMinRight, m_yMaxRight, 2);
    
    update();
}

void ChartWidget::setTimeRange(int seconds)
{
    m_timeRange = seconds;
    update();
}

void ChartWidget::updateChart()
{
    update();
}

void ChartWidget::onDataUpdated(const SensorData &data)
{
    // 直接添加新数据点（事件驱动）
    m_dataBuffer.append(data);
    
    // 限制缓冲区大小
    if (m_dataBuffer.size() > m_maxPoints) {
        m_dataBuffer.removeFirst();
    }
    
    // 更新Y轴范围
    calculateDataRange(m_yMinLeft, m_yMaxLeft, 0);
    calculateDataRange(m_yMinRight, m_yMaxRight, 2);
    
    // 标记数据已更新
    m_dataUpdated = true;
    
    // 关键修复：使用repaint()立即触发重绘，不等待事件队列
    // update()会将重绘事件放入队列，可能被合并导致延迟
    repaint();
}

void ChartWidget::onUpdateTimer()
{
    // 强制每50ms刷新一次，确保20Hz刷新率
    // 使用repaint()立即重绘，避免Qt合并重绘事件导致的延迟
    repaint();
    
    // 帧率统计（每100帧打印一次）
    if (++m_frameCount >= 100) {
        double elapsed = m_statsTimer->elapsed() / 1000.0;
        double fps = m_frameCount / elapsed;
        qDebug() << "Chart FPS:" << QString::number(fps, 'f', 1) << "Hz";
        m_frameCount = 0;
        m_statsTimer->restart();
    }
    
    // 重置数据更新标志
    m_dataUpdated = false;
}

void ChartWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制标题
    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 14, QFont::Bold));
    painter.drawText(rect().adjusted(0, 5, 0, 0), Qt::AlignHCenter | Qt::AlignTop, "传感器数据曲线");
    
    // 更新绘图区域
    m_plotRect = QRect(m_marginLeft, m_marginTop, 
                       width() - m_marginLeft - m_marginRight,
                       height() - m_marginTop - m_marginBottom);
    
    // 绘制坐标轴
    drawAxes(painter);
    
    // 根据显示类型绘制数据曲线
    if (m_displayType == 0 || m_displayType == 1) {
        drawSeries(painter, 1, Qt::blue);  // 压力
    }
    if (m_displayType == 0 || m_displayType == 2) {
        drawSeries(painter, 2, Qt::green); // 绳长
    }
    if (m_displayType == 0 || m_displayType == 3) {
        drawSeries(painter, 3, Qt::red);   // 电流
    }
    if (m_displayType == 0 || m_displayType == 4) {
        drawSeries(painter, 4, Qt::magenta); // 电压
    }
    
    // 绘制图例
    int legendX = m_plotRect.right() - 150;
    int legendY = m_marginTop + 10;
    painter.setFont(QFont("Arial", 10));
    
    if (m_displayType == 0 || m_displayType == 1) {
        painter.setPen(Qt::blue);
        painter.drawText(legendX, legendY, "压力(kg)");
    }
    if (m_displayType == 0 || m_displayType == 2) {
        painter.setPen(Qt::green);
        painter.drawText(legendX, legendY + 15, "绳长(m)");
    }
    if (m_displayType == 0 || m_displayType == 3) {
        painter.setPen(Qt::red);
        painter.drawText(legendX, legendY + 30, "电流(A)");
    }
    if (m_displayType == 0 || m_displayType == 4) {
        painter.setPen(Qt::magenta);
        painter.drawText(legendX, legendY + 45, "电压(V)");
    }
}

void ChartWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    m_plotRect = QRect(m_marginLeft, m_marginTop, 
                       width() - m_marginLeft - m_marginRight,
                       height() - m_marginTop - m_marginBottom);
}

void ChartWidget::drawAxes(QPainter &painter)
{
    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 10));
    
    // 绘制边框
    painter.drawRect(m_plotRect);
    
    // 绘制X轴刻度和标签
    int numXTicks = 7;
    for (int i = 0; i < numXTicks; ++i) {
        int x = m_plotRect.left() + (m_plotRect.width() * i / (numXTicks - 1));
        double time = m_timeRange * i / (numXTicks - 1);
        
        // 刻度线
        painter.drawLine(x, m_plotRect.bottom(), x, m_plotRect.bottom() + 5);
        
        // 标签
        painter.drawText(x - 15, m_plotRect.bottom() + 20, QString::number(time, 'f', 0) + "s");
    }
    
    // X轴标题
    painter.drawText(m_plotRect.center().x() - 20, height() - 10, "时间(s)");
    
    // 绘制左侧Y轴（压力、绳长）
    int numYTicks = 6;
    for (int i = 0; i < numYTicks; ++i) {
        int y = m_plotRect.bottom() - (m_plotRect.height() * i / (numYTicks - 1));
        double value = m_yMinLeft + (m_yMaxLeft - m_yMinLeft) * i / (numYTicks - 1);
        
        // 刻度线
        painter.drawLine(m_plotRect.left() - 5, y, m_plotRect.left(), y);
        
        // 标签
        painter.drawText(m_plotRect.left() - 55, y + 4, QString::number(value, 'f', 1));
    }
    
    // 左侧Y轴标题
    painter.save();
    painter.translate(15, m_plotRect.center().y());
    painter.rotate(-90);
    painter.drawText(-40, 0, "压力(kg)/绳长(m)");
    painter.restore();
    
    // 绘制右侧Y轴（电流、电压）
    for (int i = 0; i < numYTicks; ++i) {
        int y = m_plotRect.bottom() - (m_plotRect.height() * i / (numYTicks - 1));
        double value = m_yMinRight + (m_yMaxRight - m_yMinRight) * i / (numYTicks - 1);
        
        // 刻度线
        painter.drawLine(m_plotRect.right(), y, m_plotRect.right() + 5, y);
        
        // 标签
        painter.drawText(m_plotRect.right() + 8, y + 4, QString::number(value, 'f', 1));
    }
    
    // 右侧Y轴标题
    painter.save();
    painter.translate(width() - 15, m_plotRect.center().y());
    painter.rotate(90);
    painter.drawText(-40, 0, "电流(A)/电压(V)");
    painter.restore();
    
    // 绘制网格线
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));
    for (int i = 1; i < numXTicks - 1; ++i) {
        int x = m_plotRect.left() + (m_plotRect.width() * i / (numXTicks - 1));
        painter.drawLine(x, m_plotRect.top(), x, m_plotRect.bottom());
    }
    for (int i = 1; i < numYTicks - 1; ++i) {
        int y = m_plotRect.bottom() - (m_plotRect.height() * i / (numYTicks - 1));
        painter.drawLine(m_plotRect.left(), y, m_plotRect.right(), y);
    }
}

void ChartWidget::drawSeries(QPainter &painter, int seriesType, const QColor &color)
{
    if (m_dataBuffer.isEmpty()) {
        return;
    }
    
    painter.setPen(QPen(color, 2));
    
    QPainterPath path;
    bool firstPoint = true;
    
    double currentTime = m_startTime.secsTo(m_dataBuffer.last().timestamp);
    double startTime = qMax(0.0, currentTime - m_timeRange);
    
    // 使用指针访问数据以提高性能
    const SensorData *dataPtr = m_dataBuffer.constData();
    int dataSize = m_dataBuffer.size();
    
    for (int i = 0; i < dataSize; ++i) {
        double time = m_startTime.secsTo(dataPtr[i].timestamp);
        if (time < startTime) {
            continue;
        }
        
        // 根据类型获取实际值
        double value = 0;
        switch (seriesType) {
        case 1: value = dataPtr[i].pressure; break;
        case 2: value = dataPtr[i].ropeLength; break;
        case 3: value = dataPtr[i].current; break;
        case 4: value = dataPtr[i].voltage; break;
        }
        
        QPointF point = dataToScreen(time, value, seriesType);
        
        if (firstPoint) {
            path.moveTo(point);
            firstPoint = false;
        } else {
            path.lineTo(point);
        }
    }
    
    // 一次性绘制整个路径
    painter.drawPath(path);
}

QPointF ChartWidget::dataToScreen(double time, double value, int seriesType)
{
    double currentTime = m_startTime.secsTo(m_dataBuffer.last().timestamp);
    double startTime = qMax(0.0, currentTime - m_timeRange);
    
    // X坐标
    double xRatio = (time - startTime) / m_timeRange;
    double x = m_plotRect.left() + xRatio * m_plotRect.width();
    
    // Y坐标
    double yMin, yMax;
    if (seriesType <= 2) {
        // 左侧Y轴（压力、绳长）
        yMin = m_yMinLeft;
        yMax = m_yMaxLeft;
    } else {
        // 右侧Y轴（电流、电压）
        yMin = m_yMinRight;
        yMax = m_yMaxRight;
    }
    
    double yRatio = (value - yMin) / (yMax - yMin);
    double y = m_plotRect.bottom() - yRatio * m_plotRect.height();
    
    return QPointF(x, y);
}

void ChartWidget::calculateDataRange(double &minVal, double &maxVal, int seriesType)
{
    if (m_dataBuffer.isEmpty()) {
        return;
    }
    
    minVal = 1e308;
    maxVal = -1e308;
    
    for (const auto &data : m_dataBuffer) {
        double value = 0;
        if (seriesType == 0 || seriesType == 1) {
            value = data.pressure;
            minVal = qMin(minVal, value);
            maxVal = qMax(maxVal, value);
        }
        if (seriesType == 0 || seriesType == 2) {
            value = data.ropeLength;
            minVal = qMin(minVal, value);
            maxVal = qMax(maxVal, value);
        }
        if (seriesType == 2 || seriesType == 3) {
            value = data.current;
            minVal = qMin(minVal, value);
            maxVal = qMax(maxVal, value);
        }
        if (seriesType == 2 || seriesType == 4) {
            value = data.voltage;
            minVal = qMin(minVal, value);
            maxVal = qMax(maxVal, value);
        }
    }
    
    // 添加边距
    double range = maxVal - minVal;
    if (range < 0.001) {
        range = 1.0;
    }
    minVal -= range * 0.1;
    maxVal += range * 0.1;
    
    // 确保最小范围
    if (maxVal - minVal < 1.0) {
        double center = (maxVal + minVal) / 2;
        minVal = center - 0.5;
        maxVal = center + 0.5;
    }
}

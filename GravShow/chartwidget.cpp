#include "chartwidget.h"
#include <QPainter>
#include <QDebug>

ChartWidget::ChartWidget(QWidget *parent) : QWidget(parent)
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
}

ChartWidget::~ChartWidget()
{
}

void ChartWidget::addDataPoint(const SensorData &data)
{
    // 添加数据到缓冲区
    m_dataBuffer.append(data);
    
    // 限制缓冲区大小
    if (m_dataBuffer.size() > m_maxPoints) {
        m_dataBuffer.removeFirst();
    }
    
    // 更新Y轴范围
    calculateDataRange(m_yMinLeft, m_yMaxLeft, 0);  // 左侧Y轴（压力/绳长）
    calculateDataRange(m_yMinRight, m_yMaxRight, 2); // 右侧Y轴（电流/电压）
    
    // 触发重绘
    update();
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
    
    QVector<QPointF> points;
    double currentTime = m_startTime.secsTo(m_dataBuffer.last().timestamp);
    double startTime = qMax(0.0, currentTime - m_timeRange);
    
    for (const auto &data : m_dataBuffer) {
        double time = m_startTime.secsTo(data.timestamp);
        if (time < startTime) {
            continue;
        }
        
        QPointF point = dataToScreen(time, 0, seriesType);
        
        // 根据类型获取实际值
        double value = 0;
        switch (seriesType) {
        case 1: value = data.pressure; break;
        case 2: value = data.ropeLength; break;
        case 3: value = data.current; break;
        case 4: value = data.voltage; break;
        }
        
        point = dataToScreen(time, value, seriesType);
        points.append(point);
    }
    
    // 绘制折线
    if (points.size() > 1) {
        for (int i = 0; i < points.size() - 1; ++i) {
            painter.drawLine(points[i], points[i + 1]);
        }
    }
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

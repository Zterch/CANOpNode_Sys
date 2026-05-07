#ifndef CHARTWIDGET_H
#define CHARTWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QVector>
#include "datamodel.h"

/**
 * @brief 曲线绘制控件 - 使用QPainter绘制传感器数据曲线
 */
class ChartWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ChartWidget(QWidget *parent = nullptr);
    ~ChartWidget();

    // 添加数据点
    void addDataPoint(const SensorData &data);
    
    // 清空数据
    void clearData();
    
    // 设置显示的数据类型
    void setDisplayType(int type);  // 0=全部, 1=压力, 2=绳长, 3=电流, 4=电压
    
    // 从历史数据加载并显示
    void loadFromHistory(const QVector<SensorData> &history);
    
    // 设置X轴范围（秒）
    void setTimeRange(int seconds);

public slots:
    // 更新显示
    void updateChart();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    // 绘制坐标轴
    void drawAxes(QPainter &painter);
    
    // 绘制数据曲线
    void drawSeries(QPainter &painter, int seriesType, const QColor &color);
    
    // 数据值到屏幕坐标的转换
    QPointF dataToScreen(double time, double value, int seriesType);
    
    // 计算数据的范围
    void calculateDataRange(double &minVal, double &maxVal, int seriesType);

private:
    // 数据存储
    QVector<SensorData> m_dataBuffer;
    int m_maxPoints = 2000;  // 最大显示点数
    int m_timeRange = 60;    // 默认显示60秒
    int m_displayType = 0;   // 显示类型
    
    // 绘图区域
    QRect m_plotRect;
    int m_marginLeft = 60;
    int m_marginRight = 60;
    int m_marginTop = 40;
    int m_marginBottom = 50;
    
    // 开始时间
    QDateTime m_startTime;
    
    // Y轴范围缓存
    double m_yMinLeft = -1.0, m_yMaxLeft = 5.0;
    double m_yMinRight = 0.0, m_yMaxRight = 10.0;
};

#endif // CHARTWIDGET_H

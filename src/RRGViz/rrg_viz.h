#ifndef RRG_VIZ_H_
#define RRG_VIZ_H_

#include <QLabel>

#include "rrg.h"
#include "path_planning_info.h"

class RRGViz : public QLabel
{
    Q_OBJECT
public:
    explicit RRGViz(QWidget *parent = 0);
    void setTree(RRG* p_tree);
    bool drawPath(QString filename);

    PathPlanningInfo m_PPInfo;
signals:
    
public slots:

private:
    void drawPathOnMap(QPixmap& map);
    RRG* mp_tree;

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // RRG_VIZ_H_

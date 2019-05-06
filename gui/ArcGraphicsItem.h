#pragma once

#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/PainterOstream.h>

#include <QPen>

#include "gml/BasicInput.h"
#include "Wavefront.h"
#include "cgTypes.h"

class ArcGraphicsItem :
  public CGAL::Qt::GraphicsItem
{
  private:
    using Base = CGAL::Qt::GraphicsItem;

  private:
    const Nodes * const nodes;
    const ArcList * const arcs;
    const std::vector<Edge> * const lines;
    CGAL::Qt::PainterOstream<K> painterostream;
    QPen vertices_pen;
    QPen segments_pen;
    QPen segments_debug_pen;
    QPen labels_pen;

    bool visible_labels 	 = false;
    bool visible_arc_labels  = false;
    bool visible_node_labels = false;

    bool drawNode(const Node& node) const;

  protected:
    QRectF bounding_rect;
    void updateBoundingBox();

  public:
    ArcGraphicsItem(const Nodes * const nodes, const ArcList * arcs, const std::vector<Edge> * lines);

    QRectF boundingRect() const { return bounding_rect; };

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void setVerticesPen(const QPen& pen) { vertices_pen = pen; };
    void setSegmentsPen(const QPen& pen) { segments_pen = pen; };
    void setLabelsPen(const QPen& pen) { labels_pen = pen; };
    const QPen& verticesPen() const { return vertices_pen; }
    const QPen& segmentsPen() const { return segments_pen; }
    const QPen& segmentsDebugPen() const { return segments_debug_pen; }
    const QPen& labelsPen() const { return labels_pen; }
    void setVisibleLabels(bool visible) { if (visible_labels != visible) { prepareGeometryChange(); }; visible_labels = visible; }
    void setVisibleArcLabels(bool visible) { if (visible_arc_labels != visible) { prepareGeometryChange(); }; visible_arc_labels = visible; }
    void setVisibleNodeLabels(bool visible) { if (visible_node_labels != visible) { prepareGeometryChange(); }; visible_node_labels = visible; }

    void modelChanged();
};

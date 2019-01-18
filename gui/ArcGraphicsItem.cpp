/*
 * Copyright (c) 2015 Peter Palfrader
 *
 * All Rights reserved.
 */

#include <CGAL/Bbox_2.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Qt/Converter.h>
#include <QWidget>

#include "ArcGraphicsItem.h"
#include "cgTypes.h"

ArcGraphicsItem::
ArcGraphicsItem(const Nodes * const nodes, const ArcList * arcs)
  : Base()
  , nodes(nodes)
  , arcs(arcs)
  , painterostream(0)
  , vertices_pen(QPen(::Qt::blue, 3))
  , segments_pen(QPen(::Qt::blue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , labels_pen(QPen(Qt::darkBlue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
{
  modelChanged();
  setZValue(3);
}

void
ArcGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/) {
  CGAL::Qt::Converter<K> convert;

  painterostream = CGAL::Qt::PainterOstream<K> (painter);

  painter->setPen(segmentsPen());
  for (const auto& e : *arcs) {
	painterostream << e.edge;
  }

  painter->setPen(verticesPen());
  auto transform = painter->worldTransform();
  painter->resetTransform();
  for (const auto& i : *nodes) {
    /* Using this results in "points" that are wide rectangles when one zooms too far in,
     * so we draw out own points after manually transforming.
     * //painterostream << i;
     */

    QPointF point = transform.map(convert(i.point));
    painter->drawPoint(point);
  }
  if (visible_labels) {
    painter->setPen(labelsPen());
    QFont font(painter->font());
    /*
    //font.setPixelSize(10);
    font.setPointSize(10);
    painter->setFont(font);
    for (const auto& e : input->edges()) {
      const QPointF p(transform.map(convert( CGAL::midpoint(input->get_segment(e).source(), input->get_segment(e).target()) )));
      painter->drawText(p, "s");
    }
    */
    font.setPointSize(8);
    painter->setFont(font);
    for (auto v = nodes->begin(); v != nodes->end(); ++v) {
      const QPointF p(transform.map(convert(v->point)));
      std::string t = "v#"+std::to_string(v - nodes->begin());
      painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
    }
  }
  painter->setWorldTransform(transform);
}

void
ArcGraphicsItem::
updateBoundingBox() {
  CGAL::Qt::Converter<K> convert;
  prepareGeometryChange();

  if (nodes->size() == 0) {
    return;
  }

  auto bb = (*nodes->begin()).point.bbox();
  for (auto v : *nodes) {
    bb += v.point.bbox();
  }

  //std::cout << "bb " << bb << std::endl;
  bounding_rect = convert(bb);
}

void
ArcGraphicsItem::
modelChanged() {
  updateBoundingBox();
}

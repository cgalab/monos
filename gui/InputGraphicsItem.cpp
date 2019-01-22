/*
 * Copyright (c) 2015 Peter Palfrader
 *
 * All Rights reserved.
 */

#include <CGAL/Bbox_2.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Qt/Converter.h>
#include <QWidget>

#include <string>

#include "InputGraphicsItem.h"
#include "cgTypes.h"

InputGraphicsItem::
InputGraphicsItem(const BasicInput * const input)
  : Base()
  , input(input)
  , painterostream(0)
  , vertices_pen(QPen(::Qt::black, 3))
  , segments_pen(QPen(::Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
  , labels_pen(QPen(Qt::black, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
{
  modelChanged();
  setZValue(3);
}

void
InputGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/) {
	CGAL::Qt::Converter<K> convert;

	painterostream = CGAL::Qt::PainterOstream<K> (painter);

	painter->setPen(segmentsPen());
	for (const auto& e : input->edges()) {
		painterostream << input->get_segment(e);
	}

	painter->setPen(verticesPen());
	auto transform = painter->worldTransform();
	painter->resetTransform();
	for (const auto& i : input->vertices()) {
		/* Using this results in "points" that are wide rectangles when one zooms too far in,
		 * so we draw out own points after manually transforming.
		 * //painterostream << i;
		 */

		QPointF point = transform.map(convert(i.p));
		painter->drawPoint(point);
	}
	if (visible_edge_labels) {
		painter->setPen(labelsPen());
		QFont font(painter->font());

		font.setPointSize(10);
		painter->setFont(font);
		for (auto e = input->edges().begin(); e != input->edges().end(); ++e) {
			const QPointF p(transform.map(convert( CGAL::midpoint(input->get_segment(*e).source(), input->get_segment(*e).target()) )));
			std::string t = "e#"+std::to_string(e - input->edges().begin());
			if(e->weight != CORE_ONE) {
				t += "(" + std::to_string(e->weight.doubleValue()) + ")";
			}
			painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
		}
	}
	if (visible_labels) {
		painter->setPen(labelsPen());
		QFont font(painter->font());

		font.setPointSize(8);
		painter->setFont(font);
		for (const auto& v : input->vertices()) {
			const QPointF p(transform.map(convert(v.p)));
			std::string t = "v#"+std::to_string(v.id);
			painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
		}
	}
	painter->setWorldTransform(transform);
}

void
InputGraphicsItem::
updateBoundingBox() {
  CGAL::Qt::Converter<K> convert;
  prepareGeometryChange();

  if (input->vertices().size() == 0) {
    return;
  }

  auto bb = (*input->vertices().begin()).p.bbox();
  for (auto& v : input->vertices()) {
    bb += v.p.bbox();
  }

  //std::cout << "bb " << bb << std::endl;
  bounding_rect = convert(bb);
}

void
InputGraphicsItem::
modelChanged() {
  updateBoundingBox();
}

/**
 *  Copyright 2015 -- 2019 Peter Palfraader
 *              2018, 2019 Günther Eder - geder@cs.sbg.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <CGAL/Bbox_2.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Qt/Converter.h>
#include <QWidget>

#include <string>

#include "InputGraphicsItem.h"
#include "cgTypes.h"
#include "BasicInput.h"

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
		for (ul i = 0; i < input->edges().size(); ++i) {
			const auto e = input->get_segment(input->get_edge(i));
			const QPointF p(transform.map(convert( CGAL::midpoint(e.source(), e.target()) )));
			std::string t = "e#"+std::to_string(i);
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

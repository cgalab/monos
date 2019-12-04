/**
 *  Copyright 2018, 2019 Peter Palfraader
 *            2018, 2019 Günther Eder - geder@cs.sbg.ac.at
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
  , segments_debug_pen(QPen(::Qt::magenta, 0, Qt::DashDotDotLine, Qt::RoundCap, Qt::RoundJoin))
  , labels_pen(QPen(Qt::darkBlue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin))
{
  modelChanged();
  setZValue(3);
}


bool ArcGraphicsItem::drawNode(const Node& node) const {
	if( node.isDisabled() || node.arcs.empty() || node.arcs.size() < 2) {
		return false;
	}
	for(auto a : node.arcs) {
		assert(a < arcs->size());
		if( (*arcs)[a].type == ArcType::NORMAL ||(*arcs)[a].type == ArcType::RAY) {
			return true;
		}
	}
	return false;
}

void ArcGraphicsItem::
paint(QPainter *painter, const QStyleOptionGraphicsItem * /*option*/, QWidget * /*widget*/) {
	CGAL::Qt::Converter<K> convert;

	painterostream = CGAL::Qt::PainterOstream<K> (painter);

	painter->setPen(segmentsPen());
	for (const auto& e : *arcs) {
		if(e.type != ArcType::DISABLED) {
			painterostream << e;
		}
	}

	painter->setPen(verticesPen());
	auto transform = painter->worldTransform();
	painter->resetTransform();
	for (const auto& i : *nodes) {
		/* Using this results in "points" that are wide rectangles when one zooms too far in,
		 * so we draw out own points after manually transforming.
		 * //painterostream << i;
		 */

		if(drawNode(i)) {
			QPointF point = transform.map(convert(i.point));
			painter->drawPoint(point);
		}
	}

	if (visible_arc_labels) {
		painter->setPen(labelsPen());
		QFont font(painter->font());

		font.setPointSize(10);
		painter->setFont(font);
		for (auto e = arcs->begin(); e != arcs->end(); ++e) {
			if(e->type != ArcType::DISABLED) {
				QPointF p(transform.map(convert( CGAL::midpoint(e->source(), e->target()) )));
				std::string t = "a#"+std::to_string(e - arcs->begin());
				painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
			}
		}
	}

	if(visible_node_labels) {
		painter->setPen(labelsPen());
		QFont font(painter->font());
		font.setPointSize(8);
		painter->setFont(font);
		for (auto v = nodes->begin(); v != nodes->end(); ++v) {
			if(drawNode(*v)) {
				const QPointF p(transform.map(convert(v->point)));
				std::string t = "n#"+std::to_string(v - nodes->begin());
				painter->drawText(p.x()+4, p.y(), QString::fromStdString(t));
			}
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

//  auto bb = (*nodes->begin()).point.bbox();
//  for (auto v : *nodes) {
//	  if(!v.isDisabled() && v.point != INFPOINT) {
//		  bb += v.point.bbox();
//	  }
//  }
//  bounding_rect = convert(bb);
}

void
ArcGraphicsItem::
modelChanged() {
  updateBoundingBox();
}

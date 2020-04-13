#include "gridnode.h"

GridNode::GridNode(Vector3f position):
    m_position(position)
{

}

Vector3f GridNode::getPosition() {
    return m_position;
}

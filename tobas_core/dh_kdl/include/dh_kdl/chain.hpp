#pragma once

#include <string>

#include "./segment.hpp"

namespace KDL
{
/**
 * \brief This class encapsulates a <strong>serial</strong> kinematic
 * interconnection structure. It is built out of segments.
 *
 * @ingroup KinematicFamily
 */
class Chain
{
public:
  std::vector<Segment> segments;

  /**
   * The constructor of a chain, a new chain is always empty.
   */
  explicit Chain();

  /* Copy constructor */
  explicit Chain(const Chain& in);
  Chain& operator=(const Chain& arg);

  /**
   * Adds a new segment to the <strong>end</strong> of the chain.
   *
   * @param segment The segment to add
   */
  void addSegment(const Segment& segment);

  /**
   * Adds a complete chain to the <strong>end</strong> of the chain
   * The added chain is copied.
   *
   * @param chain The chain to add
   */
  void addChain(const Chain& chain);

  /**
   * Request the total number of joints in the chain.\n
   * <strong> Important:</strong> It is not the
   * same as the total number of segments since a segment does not
   * need to have a joint. This function is important when
   * creating a KDL::JntArray to use with this chain.
   * @return total nr of joints
   */
  inline const size_t& getNrOfJoints() const;

  /**
   * Request the total number of segments in the chain.
   * @return total number of segments
   */
  inline const size_t& getNrOfSegments() const;

  /**
   * Request the nr'd segment of the chain. There is no boundary
   * checking.
   *
   * @param nr the nr of the segment starting from 0
   *
   * @return a constant reference to the nr'd segment
   */
  inline const Segment& getSegment(size_t nr) const;

  /**
   * Request the nr'd segment of the chain. There is no boundary
   * checking.
   *
   * @param nr the nr of the segment starting from 0
   *
   * @return a reference to the nr'd segment
   */
  inline Segment& getSegment(size_t nr);

private:
  size_t nj_;  // The number of joints
  size_t ns_;  // The number of segments
};

inline const size_t& Chain::getNrOfJoints() const
{
  return nj_;
};

inline const size_t& Chain::getNrOfSegments() const
{
  return ns_;
};

inline const Segment& Chain::getSegment(size_t nr) const
{
  return segments[nr];
}

inline Segment& Chain::getSegment(size_t nr)
{
  return segments[nr];
}
}  // namespace KDL

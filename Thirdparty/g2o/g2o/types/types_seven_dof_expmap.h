// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Raúl Mur Artal (2014)
// - Added EdgeInverseSim3ProjectXYZ 
// - Modified VertexSim3Expmap to represent relative transformation between two cameras. Includes calibration of both cameras.

#ifndef G2O_SEVEN_DOF_EXPMAP_TYPES
#define G2O_SEVEN_DOF_EXPMAP_TYPES

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "types_six_dof_expmap.h"
#include "sim3.h"

namespace g2o {

  using namespace Eigen;

  /**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexSim3Expmap : public BaseVertex<7, Sim3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate = Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
      Eigen::Map<Vector7d> update(const_cast<double*>(update_));

      if (_fix_scale)
        update[6] = 0;

      Sim3 s(update);
      setEstimate(s*estimate());
    }

    Vector2d _principle_point1, _principle_point2;
    Vector2d _focal_length1, _focal_length2;

    Vector2d cam_map1(const Vector2d & v) const
    {
      Vector2d res;
      res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
      res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
      return res;
    }

    Vector2d cam_map2(const Vector2d & v) const
    {
      Vector2d res;
      res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
      res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
      return res;
    }

    bool _fix_scale;


  protected:
  };

  /**
 * \brief 7D edge between two Vertex7
 */
  class EdgeSim3 : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
      const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

      Sim3 C(_measurement);
      Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
      _error = error_.log();
    }

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *) { return 1.; }
    virtual void initialEstimate(const OptimizableGraph::VertexSet &from, OptimizableGraph::Vertex * /*to*/)
    {
      VertexSim3Expmap *v1 = static_cast<VertexSim3Expmap *>(_vertices[0]);
      VertexSim3Expmap *v2 = static_cast<VertexSim3Expmap *>(_vertices[1]);
      if (from.count(v1) > 0)
        v2->setEstimate(measurement() * v1->estimate());
      else
        v1->setEstimate(measurement().inverse() * v2->estimate());
    }
  };

  /**
 * \brief 7D edge between two Vertex when GeoSLAM
 */
  class EdgeSim3InGeoSLAM : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSE3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3InGeoSLAM(){};

    virtual bool read(std::istream& is){};
    virtual bool write(std::ostream& os) const{};
    void computeError()//计算误差
    {
      const VertexSim3Expmap *v1 = static_cast<const VertexSim3Expmap *>(_vertices[0]);
      const VertexSE3Expmap *v2 = static_cast<const VertexSE3Expmap *>(_vertices[1]);

      Sim3 Sgw = v1->estimate();
      SE3Quat Tcw = v2->estimate();
      Sim3 Scw(Tcw.rotation(), Tcw.translation(), 1);

      Eigen::Quaterniond r_tmp;
      r_tmp.setIdentity();
      Eigen::Vector3d t_tmp;
      t_tmp.fill(0.);
      
      Sim3 Scc(r_tmp, t_tmp,Sgw.scale());

      Sim3 Scg(_measurement);
      Sim3 error_ = Scg * Sgw * Scw.inverse() * Scc.inverse();
      _error = error_.log();
    };
    void linearizeOplus() // 计算雅可比
    {
      const VertexSim3Expmap *v1 = static_cast<const VertexSim3Expmap *>(_vertices[0]);
      const VertexSE3Expmap *v2 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
      Sim3 Sgw = v1->estimate();
      SE3Quat Tcw = v2->estimate();
      Sim3 Scw(Tcw.rotation(), Tcw.translation(), 1);
      Eigen::Quaterniond r_tmp;
      r_tmp.setIdentity();
      Eigen::Vector3d t_tmp;
      t_tmp.fill(0.);
      Sim3 Scc(r_tmp, t_tmp, Sgw.scale());
      Sim3 Scg(_measurement);
      Sim3 error_ = Scg * Sgw * Scw.inverse() * Scc.inverse();
      Sim3 Scc_tmp = Scg * Sgw * Scw.inverse() ;

      const Eigen::Matrix<double, 7, 7> I7 = Eigen::Matrix<double, 7, 7>::Identity();

      Eigen::Matrix<double, 7, 7> adj_Scg = I7;
      adj_Scg.block<3, 3>(0, 0) = Scg.rotation().toRotationMatrix();
      adj_Scg.block<3, 3>(3, 3) = Scg.scale() * Scg.rotation().toRotationMatrix();
      adj_Scg.block<3, 3>(3, 0) = skew(Scg.translation()) * Scg.rotation().toRotationMatrix();
      adj_Scg.block<3, 1>(3, 6) = -Scg.translation();

      Eigen::Matrix<double, 7, 7> adj_error_ = I7;
      adj_error_.block<3, 3>(0, 0) = error_.rotation().toRotationMatrix();
      adj_error_.block<3, 3>(3, 3) = error_.scale() * error_.rotation().toRotationMatrix();
      adj_error_.block<3, 3>(3, 0) = skew(error_.translation()) * error_.rotation().toRotationMatrix();
      adj_error_.block<3, 1>(3, 6) = -error_.translation();

      _jacobianOplusXi = adj_Scg;
      _jacobianOplusXi(6, 6) = _jacobianOplusXi(6, 6) - adj_error_(6, 6);

      Eigen::Matrix<double, 7, 7> adj_Scc_tmp = I7;
      adj_Scc_tmp.block<3, 3>(0, 0) = Scc_tmp.rotation().toRotationMatrix();
      adj_Scc_tmp.block<3, 3>(3, 3) = Scc_tmp.scale() * Scc_tmp.rotation().toRotationMatrix();
      adj_Scc_tmp.block<3, 3>(3, 0) = skew(Scc_tmp.translation()) * Scc_tmp.rotation().toRotationMatrix();
      adj_Scc_tmp.block<3, 1>(3, 6) = -Scc_tmp.translation();
      
      _jacobianOplusXj = -adj_Scc_tmp.block<7,6>(0,0);
    };
  };

  /**/
  class EdgeSim3ProjectXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
      const VertexSim3Expmap *v1 = static_cast<const VertexSim3Expmap *>(_vertices[1]);
      const VertexSBAPointXYZ *v2 = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);

      Vector2d obs(_measurement);
      _error = obs - v1->cam_map1(project(v1->estimate().map(v2->estimate())));
    }

    // virtual void linearizeOplus();
  };

  /**/
  class EdgeInverseSim3ProjectXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Vector2d obs(_measurement);
      _error = obs-v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())));
    }

   // virtual void linearizeOplus();
  };

} // end namespace

#endif


---
layout: page
title: auto Inverse Kinematics solver for 6R robotic arms
description: 
img: assets/img/iksolver.png
importance: 2
category: robotics
related_publications: true
---

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/iksolver.png" title="IK software" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Based on Gibbs quaternions and directional tangent matrices, our team developed a vector polynomial solving system to achieve real-time inverse kinematics solutions for a bionic robot arm with approximately parallel axes.

<style>
  .code-preview-container {
    position: relative;
    max-height: 150px; /* show ~3-4 lines */
    overflow: hidden;
  }

  .code-preview-container.open {
    max-height: none;
  }

  .expand-btn {
    display: inline-block;
    margin-top: 8px;
    color: #007acc;
    cursor: pointer;
    font-size: 0.9rem;
  }
</style>



Due to the limited numerical precision of certain open-source libraries, such as Eigen, we implemented a C++-based solution that leverages symbolic computation to numerically solve high-order univariate equations, ensuring both high accuracy and computational efficiency in engineering contexts.

In Vector_template.hpp, I implemented a template for vectors and matrices to handle memory management, construction and destruction, deep and shallow copying, as well as basic operations.
<div id="code-block1" class="code-preview-container">
  {% highlight cpp %}
  #pragma once
#include "../util/BaseException.hpp"
///////////////////////Isomorphism between mathematic spaces//////////////////////////////////////////////////
//(A1) The closure of addition: if a and b belong to S, then a+b=S
//(A2) The union law of addition: for any element a,b,c in S, a+(b+c)=(a+b)+c
//(A3) Addition unit element: there exists an element o in R such that for any element a in S, a + o = o + a = a
//(A4) Additive inverse element: for any element a in S, there must exist an element -a in S such that a + (-a) = (-a) + a = 0
//[A1->A4]:group (+- into groups)
//(A5) Law of additive exchange: for any elements a and b in S, we have a + b = b + a
//[A1->A5]:Exchangeable groups/abelian(+-exchangeable)
//(M1) Closure of multiplication:if a and b belong to S, then ab belongs to S
//(M2) Multiplicative union: for any element a,b,c in S, there is a*(b*c) = (a*b)*c
//(M3) distributive law: for any element a,b,c in S, we have a*(b+c)=a*b+a*c and (a+b)*c=a*c+b*c
//[A1-M3]:ring(+-*,ring city)
//(M4) Multiplicative exchange law:for any elements a and b in S, a*b = b*a
//[A1-M4]:Exchangeable rings
//(M5) Multiplicative unit element: for any element a in S, there exists an element i in S,
//	such that a*i = i*a = a
//(M6) Zero-free factor: for elements a,b in S, if ab=0, then there is a=0 or b=0
//[A1-M6]: the whole ring
//(M7) Multiplicative inverse element: if a belongs to S,and a is not 0, 
//      then there exists an element a^-1 ,all in S with aa^-1=a^-1 a=1
//[A1-M7]: domain (*invertible)
////////////////////// memory management ///////////////////////////////////////////////////////////
//// [1] Intelligent management; ///////////////////////////////////////////////////////////////////
//// [2] Reducing space construction and destructuring through space reuse and referencing//////////
//// [3] Reducing data copying/////////////////////////////////////////////////////////////////////
////////////////////// Data production and consumption, pipeline ///////////////////////////////////
namespace MAS {
	//////////////////////////////////////////
	template <class T> class MatrixTemplt;
	template <class T> class PVec;
	template <class T> class PMat;
	template <class T> class PolyFun;
	class RVec;
	class RVec3;
	class RMat;
	class RMat33;
	class SVDC;
	class ReplacedTerm;
	class MultiVarTerm;
	class MAS_FK;
	class AxisParams;
#ifdef MAS_INCUDE_IK
	class PolyBaseC;
	class VecPolySysC;
	class RoboArmIKC;
	class RoboWristIKC;
#endif
	class MASDynamicsC;
	class JTPos6RSEQ;
	class OrderingSolns;
	class EularQuat;
	class Quaternion;
	class JuGibbsQuat;
	/////////Functions cannot be distinguished by their return values//////////////////////
	///////////////Automatically be typecast upwards///////////////////////////////////////
	///Establish a One-to-one handshake mechanism for pointers, classes, and other state///
	template <class T>
	class VectorTemplt : public MASClass
	{
	public:
		friend class MatrixTemplt<T>;
		friend class PVec<T>;
		friend class PMat<T>;
		friend class PolyFun<T>;
		friend class RVec;
		friend class RVec3;
		friend class RMat;
		friend class RMat33;
		friend class SVDC;
		friend class ReplacedTerm;
		friend class MultiVarTerm;
		friend class VecPolySysC;
		friend class MAS_FK;
#ifdef MAS_INCUDE_IK
		friend class AxisParams;
		friend class PolyBaseC;
		friend class RoboArmIKC;
		friend class RoboWristIKC;
#endif
		friend class MASDynamicsC;
		friend class JTPos6RSEQ;
		friend class OrderingSolns;
		friend class EularQuat;
		friend class Quaternion;
		friend class JuGibbsQuat;
	////////////////////////////////////////////////////
	public:
		VectorTemplt();
		virtual ~VectorTemplt();
		VectorTemplt(const std::vector<T>& Vec);
		VectorTemplt(const VectorTemplt<T >& Vec);
		VectorTemplt(const VectorTemplt<T >&& Vec);
		VectorTemplt(Integer size);
		////////////////////////////////////////////////////////////////////////
		void MoveFrm(const VectorTemplt<T >& Vec);
		void AssignFrm(const VectorTemplt<T >& Vec);
		const VectorTemplt<T >& operator=(const VectorTemplt<T >& Vec);
		//////////////////////////////////////////////////////////////////////////
		inline T& operator[](Integer index);
		inline const T& operator[](Integer index) const;
		inline T& At(Integer index);
		inline const T& At(Integer index) const;
		//////////////////////////////////////////////////////////////////////////
		void SetNull();
		virtual void SetSize(Integer size, bool bSetZero = true);
		inline Integer GetSize() const;
		VectorTemplt<T> GetFromTo(Integer start, Integer To);
		////////////////inline should be nested////////////////////////////////////
		inline T GetSum() const;
		inline T GetMaxCwiseAbs()const;
		inline const T InnerProduct(const VectorTemplt<T >& Vec);
		inline const T InnerProduct(const VectorTemplt<T >& Vec1, const VectorTemplt<T >& Vec2);
		///////////////////////////////////////////////////////////////////////////////////////
		void Add(const VectorTemplt<T>& Vec);	
		void Subtract(const VectorTemplt<T>& Vec);
		////////////////Minus sign/////////////////
		const VectorTemplt<T > operator-() const;
		////////////////Subtraction///////////////////////////////////////
		const VectorTemplt<T > operator-(const VectorTemplt<T >& Vec) const;
		////////////////Addition///////////////////////////////////////////
		const VectorTemplt<T > operator+(const VectorTemplt<T >& Vec) const; 
		///////////friend function globally visible//////Binary operator////////////////////////////////
		friend const VectorTemplt<T > operator+(const T& Scalar, const VectorTemplt<T >& Vec)
		{
			VectorTemplt<T > Result(Vec.sizeD);
			for (Integer no = 0; no < Vec.sizeD; no++) {
				Result.elemD[no] = Scalar + Vec.elemD[no];
			}
			return Result;
		};
		////////////////////////////////////////////////////////////////////////////////////////////
		friend const VectorTemplt<T > operator+(const VectorTemplt<T >& Vec, const T& Scalar)
		{
			VectorTemplt<T > Result(Vec.sizeD);
			for (Integer no = 0; no < Vec.sizeD; no++) {
				Result.elemD[no] = Vec.elemD[no] + Scalar;
			}
			return Result;
		};
		////////////////////////////////////////////////////////////////////////////////////////////
		friend const VectorTemplt<T > operator-(const T& Scalar, const VectorTemplt<T >& Vec)
		{
			VectorTemplt<T > Result(Vec.sizeD);
			for (Integer no = 0; no < Vec.sizeD; no++) {
				Result.elemD[no] = Scalar - Vec.elemD[no];
			}
			return Result;
		};
		////////////////////////////////////////////////////////////////////////////////////////////
		friend const VectorTemplt<T > operator-(const VectorTemplt<T >& lVec, const T& Scalar)
		{
			VectorTemplt<T > Result(lVec.sizeD);
			for (Integer no = 0; no < lVec.sizeD; no++) {
				Result.elemD[no] = lVec.elemD[no] - Scalar;
			}
			return Result;
		};
		////////////////////
  {% endhighlight %}
</div>
<span class="expand-btn" onclick="document.getElementById('code-block1').classList.toggle('open'); this.style.display='none';">
  Show more
</span>


As an example of using this template, we implemented Rvector3, which includes the cross product operation, its corresponding screw matrix, and the second-order screw matrix—commonly applied for zero-position axis projection in robotic systems.

<div id="code-block2" class="code-preview-container">
  {% highlight cpp %}
  
    //////////////////
	TriVec3::TriVec3() 
	{
		Vec[0].SetZero();  Vec[1].SetZero(); Vec[2].SetZero();
	};
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator=(const TriVec3& triVec)
	{
		if (this == &triVec) {
			return *this;
		}
		Vec[0].AssignFrm(triVec.Vec[0]);
		Vec[1].AssignFrm(triVec.Vec[1]);
		Vec[2].AssignFrm(triVec.Vec[2]);
		return *this;
	};
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	TriVec3::TriVec3(const TriVec3& triVec)
	{
		Vec[0].AssignFrm(triVec.Vec[0]);
		Vec[1].AssignFrm(triVec.Vec[1]);
		Vec[2].AssignFrm(triVec.Vec[2]);
	};
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	void TriVec3::AssignFrm(const TriVec3& triVec)
	{
		Vec[0].AssignFrm(triVec.Vec[0]);	Vec[1].AssignFrm(triVec.Vec[1]);
		Vec[2].AssignFrm(triVec.Vec[2]);
	};
	////////////Minus sign////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator-() const
	{
		TriVec3 Result;
		Result.Vec[0].AssignFrm(-Vec[0]);	Result.Vec[1].AssignFrm(-Vec[1]);
		Result.Vec[2].AssignFrm(-Vec[2]);
		return Result;
	};
	////////////////////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator+(const TriVec3& triVec) const
	{
		TriVec3 Result;
		Result.Vec[0].AssignFrm(Vec[0] + triVec.Vec[0]);
		Result.Vec[1].AssignFrm(Vec[1] + triVec.Vec[1]);
		Result.Vec[2].AssignFrm(Vec[2] + triVec.Vec[2]);
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator+=(const TriVec3& triVec)
	{
		Vec[0] += triVec.Vec[0];	Vec[1] += triVec.Vec[1];	Vec[2] += triVec.Vec[2];
		return *this;
	};
	///////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator-(const TriVec3& triVec) const
	{
		TriVec3 Result;
		Result.Vec[0].AssignFrm(Vec[0] - triVec.Vec[0]);
		Result.Vec[1].AssignFrm(Vec[1] - triVec.Vec[1]);
		Result.Vec[2].AssignFrm(Vec[2] - triVec.Vec[2]);
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator-=(const TriVec3& triVec)
	{
		Vec[0] -= triVec.Vec[0];	Vec[1] -= triVec.Vec[1];	Vec[2] -= triVec.Vec[2];
		return *this;
	};
	///////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator*(Real Scalar) const
	{
		TriVec3 Result;
		Result.Vec[0] *= Scalar;	Result.Vec[1] *= Scalar;	Result.Vec[2] *= Scalar;
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator*=(Real Scalar)
	{
		Vec[0] *= Scalar;	Vec[1] *= Scalar;	Vec[2] *= Scalar;
		return *this;
	};
	///////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator+(const RVec3& Vec3) const
	{
		TriVec3 Result;
		Result.Vec[0].AssignFrm(Vec[0] + Vec3);
		Result.Vec[1].AssignFrm(Vec[1] + Vec3);
		Result.Vec[2].AssignFrm(Vec[2] + Vec3);
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator+=(const RVec3& Vec3)
	{
		Vec[0] += Vec3;		Vec[1] += Vec3;		Vec[2] += Vec3;
		return *this;
	};
	///////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator-(const RVec3& Vec3) const
	{
		TriVec3 Result;
		Result.Vec[0].AssignFrm(Vec[0] - Vec3);
		Result.Vec[1].AssignFrm(Vec[1] - Vec3);
		Result.Vec[2].AssignFrm(Vec[2] - Vec3);
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator-=(const RVec3& Vec3)
	{
		Vec[0] -= Vec3;	Vec[1] -= Vec3;	Vec[2] -= Vec3;
		return *this;
	};
	///////////////////////////////////////////////////////////////////////////////////
	TriVec3 TriVec3::operator/(Real Scalar) const
	{
		TriVec3 Result;
		if (EqualDivZero(Scalar)) {
			Real Val = MASDenominatInv * Sign(Scalar);
			Result.Vec[0] *= Val;	Result.Vec[1] *= Val;	Result.Vec[2] *= Val;
		}
		else {
			Result.Vec[0] /= Scalar;	Result.Vec[1] /= Scalar;	Result.Vec[2] /= Scalar;
		}
		return Result;
	};
	///////////////////////////////////////////////////////////////////////////////////
	const TriVec3& TriVec3::operator/=(Real Scalar)
	{
		if (EqualDivZero(Scalar)) {
			Real Val = MASDenominatInv * Sign(Scalar);
			Vec[0] *= Val;	Vec[1] *= Val;	Vec[2] *= Val;
		}
		else {
			Vec[0] /= Scalar;	Vec[1] /= Scalar;	Vec[2] /= Scalar;
		}
		return *this;
	};
  {% endhighlight %}
</div>
<span class="expand-btn" onclick="document.getElementById('code-block2').classList.toggle('open'); this.style.display='none';">
  Show more
</span>





In solving high-order univariate equations, we optimize performance by storing only the coefficients corresponding to each power of the variable.

<div id="code-block3" class="code-preview-container">
  {% highlight cpp %}
namespace MAS {
////////////////////Multivariate polynomial/////////////////////////////////////////////////////////////
	template <class T>
	class PolyFun
	{
	public:
		PolyFun();
		virtual ~PolyFun();
		PolyFun(const PolyFun<T >& po);
		inline PolyFun(const PolyFun<T >&& po);
		inline void MoveFrm(const PolyFun<T >& po);
		void AssignFrm(const PolyFun<T >& po);
		const PolyFun<T >& operator=(const PolyFun<T >& po);
		///////////for PolyFun(), SetTermInfo must be attached//////////////////
		inline void SetTermInfo(const PolyFun<T >& po)
		{
			TermInfo.LevVarNum = po.TermInfo.LevVarNum;
			TermInfo.MaxRepldOrder = po.TermInfo.MaxRepldOrder;
		};
		////////////////////////////////////////////////////////////////////////
		inline void SetTermInfo(ReplacedTermInfo& Info)
		{
			TermInfo.LevVarNum = Info.LevVarNum;
			TermInfo.MaxRepldOrder = Info.MaxRepldOrder;
		};
		////////////////////////////////////////////////////////////////////////
		inline void Clear(){
			PolyMap.clear();
		};
		////////////////////////////////////////////////////////////////////////
		inline void SetZero(){
			PolyMap.clear();
		};
		/////////////////////////////////////////////////////////////////////////
		inline std::map<T, Real >& PolyFun<T >::GetPolyMap() {
			return PolyMap;
		};
		/////////////////////////////////////////////////////////////////////////
		inline void Add(const T& term, Real Ceo)
		{
			std::map<T, Real >::iterator& ItFind = PolyMap.find(term);
			if (ItFind == PolyMap.end())
				PolyMap.insert(std::make_pair(term, Ceo));
			else {
				ItFind->second += Ceo;
			}
		};
		/////////////////////////////////////////////////////////////////////////
		void Add(const T& term, Real Ceo, std::map<T, Real >& OffSet);
		//////////////////////////////////////////////////////////////////////////////////
		static PolyFun<T > GetZero();				///the Seed of polynomial addition
		static PolyFun<T > GetOne();					///the Seed of polynomial multiplication
		void SetConstant(const Real Scalar);
		void SetSingleTerm(const T& term, const Real Scalar);
		void InsertTerms2SameCeo(std::vector<T >& Vec, const Real Addend);
		void InsertTerms2SameCeo(std::vector<T >& Vec, const Real Addend, std::map<T, Real >& Offset);
		//////////////////////////////////////////////////////////////////////////////////////////////
		const PolyFun<T > operator-() const;
		bool IsExistXs(const Int8 no);
		bool operator!=(const PolyFun<T>& po);
		bool operator==(const PolyFun<T>& po);
		const PolyFun<T >& operator*=(const Real Scalar);
		const PolyFun<T >& operator*=(const PolyFun<T >& po);
		const PolyFun<T >& operator+=(const PolyFun<T >& po);
		const PolyFun<T >& operator-=(const PolyFun<T >& po);
		void MultiplybyVec(const RVec& Vec, PVec<PolyFun<T > >& Result);
		void MultiplybyMat(const RMat& Mat, PMat<PolyFun<T > >& Result);
		//////////////////////static function///////////////////////////////////////////////////////////////////
		friend PolyFun<T > operator+(const PolyFun<T >& po1, const PolyFun<T >& po2)
		{
			PolyFun<T > Result;
			if (Result.bReplacedTerm) {
				Result.SetTermInfo(po1);
			}
			SumMap(Result.PolyMap, po1.PolyMap, po2.PolyMap);
			return Result;
		};
		//////////////////////static function///////////////////////////////////////////////////////////////////
		friend PolyFun<T > operator-(const PolyFun<T >& po1, const PolyFun<T >& po2)
		{
			PolyFun<T > Result;
			if (Result.bReplacedTerm) {
				Result.SetTermInfo(po1);
			}
			MinusMap(Result.PolyMap, po1.PolyMap, po2.PolyMap);
			return Result;
		};
		//////////////////////static function///////////////////////////////////////////////////////////////////
		friend PolyFun<T > operator*(const PolyFun<T >& po1, const PolyFun<T >& po2)
		{
			PolyFun<T > Result;
			if (Result.bReplacedTerm) {
				Result.SetTermInfo(po1);
			}
#ifdef KahanMultiPoly
			Result.MoveFrm(PolyFun<T >::PolyMapFastMultiply(po1, po2, po1.TermInfo));
#else
			PolyMapMultiply(Result.PolyMap, po1.PolyMap, po2.PolyMap, po1.TermInfo);
#endif
			return Result;
		};
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		friend PolyFun<T > operator*(const PolyFun<T >& po, const Real Scalar)
		{
			PolyFun<T > Result(po);
			std::map<T, Real >::iterator It, itEnd = Result.PolyMap.end();
			for (It = Result.PolyMap.begin(); It != itEnd; It++) {
				It->second *= Scalar;			///Have been ordered
			}
			return Result;
		};
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		friend PolyFun<T > operator*(const Real Scalar, const PolyFun<T>& po)
		{
			PolyFun<T > Result(po);
			std::map<T, Real >::iterator It, itEnd = Result.PolyMap.end();
			for (It = Result.PolyMap.begin(); It != itEnd; It++) {
				It->second *= Scalar;
			}
			return Result;
		};
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		friend PVec<PolyFun<T > > operator*(const PolyFun<T >& po, const RVec& Vec)
		{
			Integer Size = Vec.GetSize();
			PVec<PolyFun<T > > Result(Size);
#ifdef _OPENMP
#pragma omp parallel for num_threads(MASCPUCoreNum) 
#endif
			for (Integer no = 0; no < Size; no++) {
				Result[no].MoveFrm(po * Vec[no]);		///Call SetTermInfo()
			}
			return Result;
		};
		///////////////////////////////////////////////////////////////////////////
  {% endhighlight %}
</div>
<span class="expand-btn" onclick="document.getElementById('code-block3').classList.toggle('open'); this.style.display='none';">
  Show more
</span>


Taking the solution of a cubic univariate equation as an example: when the constant term 
d is sufficiently small, we treat it as negligible and reduce the equation to a quadratic form. Similarly, when the cubic coefficient 
a is close to zero, we take the reciprocal of both sides and solve for 
1/x instead. This approach helps avoid significant numerical errors that can arise when converting the equation into the standard cubic form in terms of x.


<div id="code-block4" class="code-preview-container">
  {% highlight cpp %}
Int64 MonicPolyC::NormalizeMonicPoly(std::set<SameRtC>& Rts, Int64 MinMaxOrder[2], bool& bLitEndian, MonicPolyC& DerivPoly, bool& bRefine)
	{
		bLitEndian = false;
		Int64 NetOrder = GetMonicPolyOrder(MinMaxOrder);
		if (NetOrder >= RefineMinOrder) {
			bRefine = GetBigEndianDerivative(DerivPoly);
		}
		else {
			bRefine = false;
		}
		Int64 VecSize = (Int64)CoefVec.size();
		//////////////////////////////////////////////////////////////////////////////////////////////
		if (NetOrder > 1) {									///At least 1 order,i.e CoefVec.size() > 2
			bLitEndian = (10. * Abs(CoefVec[MinMaxOrder[0]]) > Abs(CoefVec[MinMaxOrder[1]]));
			if (bLitEndian) {								///LitEndian with higher priority
				Real LitEndianCeo = CoefVec[MinMaxOrder[0]];
				for (Int64 no = 0; no < VecSize; no++) {	///Infinitesimal quantities for refined roots
					CoefVec[no] /= LitEndianCeo;
				}
				CoefVec[MinMaxOrder[0]] = ONE;
			}
			else {
				Real BigEndianCeo = CoefVec[MinMaxOrder[1]];
				for (Int64 no = 0; no < VecSize; no++) {	///Infinitesimal quantities for refined roots
					CoefVec[no] /= BigEndianCeo;
				}
				CoefVec[MinMaxOrder[1]] = ONE;
			}
		}
		else if (NetOrder == 1) {
			Real BigEndianCeo = CoefVec[MinMaxOrder[1]];
			for (Int64 no = 0; no < VecSize; no++) {
				CoefVec[no] /= BigEndianCeo;
			}
			CoefVec[MinMaxOrder[1]] = ONE;
		}
		/////////////Trival solution Added///////////////
		if (MinMaxOrder[0] > 0) {
			SameRtC SameRt;				///ZERO solution
			SameRt.Insert(ZERO, Rts);	
		}
		return NetOrder;
	};
  {% endhighlight %}
</div>
<span class="expand-btn" onclick="document.getElementById('code-block4').classList.toggle('open'); this.style.display='none';">
  Show more
</span>
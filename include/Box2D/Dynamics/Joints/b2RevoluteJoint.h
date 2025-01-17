/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_REVOLUTE_JOINT_H
#define B2_REVOLUTE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
struct b2RevoluteJointDef : public b2JointDef
{
	b2RevoluteJointDef()
	{
		type = e_revoluteJoint;
		localAnchor1.Set(0.0f, 0.0f);
		localAnchor2.Set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		enableLimit = false;
		enableMotor = false;
	}

	/// Initialize the bodies, anchors, and reference angle using the world
	/// anchor.
	void Initialize(b2Body* body1, b2Body* body2, const b2Vec2& anchor);

	/// The local anchor point relative to body1's origin.
	b2Vec2 localAnchor1;

	/// The local anchor point relative to body2's origin.
	b2Vec2 localAnchor2;

	/// The body2 angle minus body1 angle in the reference state (radians).
	b2float32 referenceAngle;

	/// A flag to enable joint limits.
	bool enableLimit;

	/// The lower angle for the joint limit (radians).
	b2float32 lowerAngle;

	/// The upper angle for the joint limit (radians).
	b2float32 upperAngle;

	/// A flag to enable the joint motor.
	bool enableMotor;

	/// The desired motor speed. Usually in radians per second.
	b2float32 motorSpeed;

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	b2float32 maxMotorTorque;
};

/// A revolute joint constrains to bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
class b2RevoluteJoint : public b2Joint
{
public:
	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;

	b2Vec2 GetReactionForce() const;
	b2float32 GetReactionTorque() const;

	/// Get the current joint angle in radians.
	b2float32 GetJointAngle() const;

	/// Get the current joint angle speed in radians per second.
	b2float32 GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	/// Get the lower joint limit in radians.
	b2float32 GetLowerLimit() const;

	/// Get the upper joint limit in radians.
	b2float32 GetUpperLimit() const;

	/// Set the joint limits in radians.
	void SetLimits(b2float32 lower, b2float32 upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed in radians per second.
	void SetMotorSpeed(b2float32 speed);

	/// Get the motor speed in radians per second.
	b2float32 GetMotorSpeed() const;

	/// Set the maximum motor torque, usually in N-m.
	void SetMaxMotorTorque(b2float32 torque);

	/// Get the current motor torque, usually in N-m.
	b2float32 GetMotorTorque() const;

	//--------------- Internals Below -------------------

	b2RevoluteJoint(const b2RevoluteJointDef* def);

	void InitVelocityConstraints(const b2TimeStep& step);
	void SolveVelocityConstraints(const b2TimeStep& step);

	bool SolvePositionConstraints();

	b2Vec2 m_localAnchor1;	// relative
	b2Vec2 m_localAnchor2;
	b2Vec2 m_pivotForce;
	b2float32 m_motorForce;
	b2float32 m_limitForce;
	b2float32 m_limitPositionImpulse;

	b2Mat22 m_pivotMass;		// effective mass for point-to-point constraint.
	b2float32 m_motorMass;	// effective mass for motor/limit angular constraint.
	
	bool m_enableMotor;
	b2float32 m_maxMotorTorque;
	b2float32 m_motorSpeed;

	bool m_enableLimit;
	b2float32 m_referenceAngle;
	b2float32 m_lowerAngle;
	b2float32 m_upperAngle;
	b2LimitState m_limitState;
};

inline b2float32 b2RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

#endif

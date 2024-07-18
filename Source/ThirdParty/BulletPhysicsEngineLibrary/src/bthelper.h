#pragma once
#include "ThirdParty/BulletPhysicsEngineLibrary/BulletMinimal.h"
// Bullet scale is 1=1m, UE is 1=1cm
// So x100
#define BULLET_TO_WORLD_SCALE 100.f
#define WORLD_TO_BULLET_SCALE (1.f/BULLET_TO_WORLD_SCALE)

/**
 *
 */
class BulletHelpers
{

public:
	inline static float ToUESize(btScalar Sz)
	{
		return Sz * BULLET_TO_WORLD_SCALE;
	}
	inline static btScalar ToBtSize(float Sz)
	{
		return Sz * WORLD_TO_BULLET_SCALE;
	}
	inline static btVector3 ToBtSize(FVector Sv)
	{
		// For clarity; this is for box sizes so no offset
		return ToBtDir(Sv);
	}
	inline static FVector ToUEPos(const btVector3& V)
	{
		return FVector(V.x(), V.y(), V.z()) * BULLET_TO_WORLD_SCALE;
	}
	inline static btVector3 ToBtPos(const FVector& V)
	{
		return btVector3(V.X, V.Y, V.Z) * WORLD_TO_BULLET_SCALE;
	}
	inline static FVector ToUEDir(const btVector3& V, bool AdjustScale = true)
	{
		if (AdjustScale)
			return FVector(V.x(), V.y(), V.z()) * BULLET_TO_WORLD_SCALE;
		else
			return FVector(V.x(), V.y(), V.z());
	}
	inline static btVector3 ToBtDir(const FVector& V, bool AdjustScale = true)
	{
		if (AdjustScale)
			return btVector3(V.X, V.Y, V.Z) * WORLD_TO_BULLET_SCALE;
		else
			return btVector3(V.X, V.Y, V.Z);
	}
	inline static btVector3 ToBtMomentOfInertia(const FVector& V, bool AdjustScale = true)
	{
		if (AdjustScale)
			return btVector3(V.X, V.Y, V.Z) * WORLD_TO_BULLET_SCALE * WORLD_TO_BULLET_SCALE;
		else
			return btVector3(V.X, V.Y, V.Z);
	}
	inline static FQuat ToUE(const btQuaternion& Q)
	{
		return FQuat(Q.x(), Q.y(), Q.z(), Q.w());
	}
	inline static btQuaternion ToBt(const FQuat& Q)
	{
		return btQuaternion(Q.X, Q.Y, Q.Z, Q.W);
	}
	inline static btQuaternion ToBt(const FRotator& r)
	{
		return ToBt(r.Quaternion());
	}
	inline static FColor ToUEColour(const btVector3& C)
	{
		return FLinearColor(C.x(), C.y(), C.z()).ToFColor(true);
	}

	inline static FTransform ToUE(const btTransform& T)
	{
		const FQuat Rot = ToUE(T.getRotation());
		const FVector Pos = ToUEPos(T.getOrigin());
		return FTransform(Rot, Pos);
	}
	inline static btTransform ToBt(const FTransform& T)
	{
		return btTransform(
			ToBt(T.GetRotation()),
			ToBtPos(T.GetLocation()));
	}
};
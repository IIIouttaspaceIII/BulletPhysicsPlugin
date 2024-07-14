#pragma once
#include "CoreMinimal.h"
#include "PhysicsEngine/BodySetup.h"
/*
#include "BulletPhysicsEngineLibrary/BulletMinimal.h"
#include "BulletPhysicsEngineLibrary/src/bthelper.h"
#include "BulletPhysicsEngineLibrary/src/motionstate.h"
#include "BulletPhysicsEngineLibrary/src/BulletMain.h"
#include "BulletPhysicsEngineLibrary/debug/btdebug.h"*/
#include "Components/ShapeComponent.h"
#include <functional>
#include "Subsystems/GameInstanceSubsystem.h"
#include <ThirdParty/BulletPhysicsEngineLibrary/BulletMinimal.h>
#include <bthelper.h>
#include <motionstate.h>
#include <BulletMain.h>
#include <ThirdParty/BulletPhysicsEngineLibrary/debug/btdebug.h>
#include "BulletSubsystem.generated.h"

USTRUCT(BlueprintType)
struct Ftris
{
	GENERATED_BODY()
	FVector a;
	FVector b;
	FVector c;
	FVector d;

};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPhysicsTick, float, DeltaTime);

UCLASS()
class BULLETPHYSICSENGINE_API UBulletSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	btCollisionConfiguration* BtCollisionConfig;
	btCollisionDispatcher* BtCollisionDispatcher;
	btBroadphaseInterface* BtBroadphase;
	btConstraintSolver* BtConstraintSolver;
	btDiscreteDynamicsWorld* BtWorld;
	BulletHelpers* BulletHelpers;
	BulletDebugDraw* btdebugdraw;
	btStaticPlaneShape* plane;
	// Custom debug interface
	btIDebugDraw* BtDebugDraw;
	// Dynamic bodies
	TArray<btRigidBody*> BtRigidBodies;
	// Static colliders
	TArray<btCollisionObject*> BtStaticObjects;
	btCollisionObject* procbody;
	// Re-usable collision shapes
	TArray<btBoxShape*> BtBoxCollisionShapes;
	TArray<btSphereShape*> BtSphereCollisionShapes;
	TArray<btCapsuleShape*> BtCapsuleCollisionShapes;
	btSequentialImpulseConstraintSolver* mt;
	// Structure to hold re-usable ConvexHull shapes based on origin BodySetup / subindex / scale
	struct ConvexHullShapeHolder
	{
		UBodySetup* BodySetup;
		int HullIndex;
		FVector Scale;
		btConvexHullShape* Shape;
	};
	TArray<ConvexHullShapeHolder> BtConvexHullCollisionShapes;
	// These shapes are for *potentially* compound rigid body shapes
	struct CachedDynamicShapeData
	{
		FName ClassName; // class name for cache
		btCollisionShape* Shape;
		bool bIsCompound; // if true, this is a compound shape and so must be deleted
		btScalar Mass;
		btVector3 Inertia; // because we like to precalc this
	};
	TArray<CachedDynamicShapeData> CachedDynamicShapes;

	// This list can be edited in the level, linking to placed static actors
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	TArray<AActor*> PhysicsStaticActors1;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	TArray<AActor*> DynamicActors;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	FString text = "ra";
	// These properties can only be edited in the Blueprint
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	float PhysicsStatic1Friction = 0.6;
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	float PhysicsStatic1Restitution = 0.3;
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Bullet Physics|Objects")
	float randvar;
	// I chose not to use spinning / rolling friction in the end since it had issues!


public:

	void SetupStaticGeometryPhysics(TArray<AActor*> Actors, float Friction, float Restitution);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void AddStaticBody(AActor* player, float Friction, float Restitution, int& ID);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void AddProcBody(AActor* Body, float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void UpdateProcBody(AActor* Body, float Friction, TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d, float Restitution, int& ID, int PrevID);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void AddRigidBody(AActor* Body, float Friction, float Restitution, int& ID, float mass);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void UpdatePlayertransform(AActor* player, int ID);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void AddImpulse(int ID, FVector Impulse, FVector Location);
	typedef const std::function<void(btCollisionShape* /*SingleShape*/, const FTransform& /*RelativeXform*/)>& PhysicsGeometryCallback;
	void ExtractPhysicsGeometry(AActor* Actor, PhysicsGeometryCallback CB);

	btCollisionObject* AddStaticCollision(btCollisionShape* Shape, const FTransform& Transform, float Friction, float Restitution, AActor* Actor);


	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	bool CastRay(
		const FVector& origin,
		const FVector& direction,
		const float length);

	void ExtractPhysicsGeometry(UStaticMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

	void ExtractPhysicsGeometry(UShapeComponent* Sc, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

	void ExtractPhysicsGeometry(const FTransform& XformSoFar, UBodySetup* BodySetup, PhysicsGeometryCallback CB);

	void ExtractPhysicsGeometry(USkeletalMeshComponent* SMC, const FTransform& InvActorXform, PhysicsGeometryCallback CB);

	btCollisionShape* GetBoxCollisionShape(const FVector& Dimensions);

	btCollisionShape* GetSphereCollisionShape(float Radius);

	btCollisionShape* GetCapsuleCollisionShape(float Radius, float Height);

	btCollisionShape* GetTriangleMeshShape(TArray<FVector> a, TArray<FVector> b, TArray<FVector> c, TArray<FVector> d);


	btCollisionShape* GetConvexHullCollisionShape(UBodySetup* BodySetup, int ConvexIndex, const FVector& Scale);

	const UBulletSubsystem::CachedDynamicShapeData& GetCachedDynamicShapeData(AActor* Actor, float Mass);

	btRigidBody* AddRigidBody(AActor* Actor, const UBulletSubsystem::CachedDynamicShapeData& ShapeData, float Friction, float Restitution);

	btRigidBody* AddRigidBody(AActor* Actor, btCollisionShape* CollisionShape, btVector3 Inertia, float Mass, float Friction, float Restitution);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void StepPhysics(float DeltaSeconds, int substeps);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void SetPhysicsState(int ID, FTransform transforms, FVector Velocity, FVector AngularVelocity, FVector& Force);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void GetPhysicsState(int ID, FTransform& transforms, FVector& Velocity, FVector& AngularVelocity, FVector& Force);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	FTransform GetPhysicsTransform(int ID);
	UFUNCTION(BlueprintCallable, Category = "Bullet Physics")
	void ResetSim();

	UPROPERTY(BlueprintAssignable, Category = "Delegates")
	FOnPhysicsTick OnPhysicsTick;

	// Method to trigger the delegate
	void TriggerOnPhysicsTick(float DeltaTime);

	FVector GetVelocityAtWorldPoint(int ID, FVector WorldLocation);

protected:
	FTickerDelegate OnTickDelegate;
	FTSTicker::FDelegateHandle OnTickHandle;
private:
	void Tick(float DeltaTime);

};
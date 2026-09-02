#pragma once

#include "CoreMinimal.h"
#include "Math/RotationMatrix.h"

#include "AircraftFrameBinding.generated.h"

/** 蒙皮模型局部空间中的水平机头方向；模型 Up 固定为 +Z。 */
UENUM()
enum class EAircraftModelForwardAxis : uint8
{
	PositiveX UMETA(DisplayName = "+X"),
	PositiveY UMETA(DisplayName = "+Y"),
	NegativeX UMETA(DisplayName = "-X"),
	NegativeY UMETA(DisplayName = "-Y"),
};

/**
 * 每个模拟 LOD 唯一的坐标绑定。
 *
 * Aircraft 模型空间由蒙皮网格定义：Forward 可配置，Up 固定为 +Z，
 * Right 由 Up × Forward 推导。Body 空间是当前 LOD RootBone 对应的
 * Chaos 刚体局部空间。BodyToModelTransform 来自 RootBone 的组合参考姿态。
 */
struct AIRCRAFT_API FAircraftFrameBinding
{
public:
	bool Configure(EAircraftModelForwardAxis InModelForwardAxis, const FTransform& InBodyToModelTransform)
	{
		FTransform NormalizedTransform = InBodyToModelTransform;
		NormalizedTransform.NormalizeRotation();
		const FVector Scale = NormalizedTransform.GetScale3D();
		if (NormalizedTransform.ContainsNaN()
			|| FMath::Abs(Scale.X) <= UE_SMALL_NUMBER
			|| FMath::Abs(Scale.Y) <= UE_SMALL_NUMBER
			|| FMath::Abs(Scale.Z) <= UE_SMALL_NUMBER)
		{
			bValid = false;
			return false;
		}

		ModelForwardAxis = InModelForwardAxis;
		BodyToModelTransform = NormalizedTransform;
		bValid = true;
		return true;
	}

	void SetModelForwardAxis(EAircraftModelForwardAxis InModelForwardAxis)
	{
		ModelForwardAxis = InModelForwardAxis;
	}

	void Invalidate()
	{
		bValid = false;
	}

	bool IsValid() const
	{
		return bValid;
	}

	EAircraftModelForwardAxis GetModelForwardAxis() const
	{
		return ModelForwardAxis;
	}

	const FTransform& GetBodyToModelTransform() const
	{
		return BodyToModelTransform;
	}

	FVector GetForwardAxisModel() const
	{
		switch (ModelForwardAxis)
		{
		case EAircraftModelForwardAxis::PositiveX: return FVector::ForwardVector;
		case EAircraftModelForwardAxis::PositiveY: return FVector::RightVector;
		case EAircraftModelForwardAxis::NegativeX: return -FVector::ForwardVector;
		case EAircraftModelForwardAxis::NegativeY: return -FVector::RightVector;
		}
		checkNoEntry();
		return FVector::RightVector;
	}

	static FVector GetUpAxisModel()
	{
		return FVector::UpVector;
	}

	FVector GetRightAxisModel() const
	{
		return FVector::CrossProduct(GetUpAxisModel(), GetForwardAxisModel()).GetSafeNormal();
	}

	FVector GetForwardAxisBody() const
	{
		return ModelVectorToBody(GetForwardAxisModel()).GetSafeNormal();
	}

	FVector GetRightAxisBody() const
	{
		return ModelVectorToBody(GetRightAxisModel()).GetSafeNormal();
	}

	FVector GetUpAxisBody() const
	{
		return ModelVectorToBody(GetUpAxisModel()).GetSafeNormal();
	}

	FQuat GetControlToBodyRotation() const
	{
		const FQuat ControlToModel(FRotationMatrix::MakeFromXZ(
			GetForwardAxisModel(), GetUpAxisModel()));
		return (BodyToModelTransform.GetRotation().Inverse() * ControlToModel).GetNormalized();
	}

	FVector ModelPositionToBody(const FVector& ModelPosition) const
	{
		return BodyToModelTransform.InverseTransformPosition(ModelPosition);
	}

	FVector BodyPositionToModel(const FVector& BodyPosition) const
	{
		return BodyToModelTransform.TransformPosition(BodyPosition);
	}

	FVector ModelVectorToBody(const FVector& ModelVector) const
	{
		return BodyToModelTransform.InverseTransformVectorNoScale(ModelVector);
	}

	FVector BodyVectorToModel(const FVector& BodyVector) const
	{
		return BodyToModelTransform.TransformVectorNoScale(BodyVector);
	}

	FTransform GetBodyWorldTransform(const FTransform& ModelWorldTransform) const
	{
		return BodyToModelTransform * ModelWorldTransform;
	}

	FTransform GetModelWorldTransform(const FTransform& BodyWorldTransform) const
	{
		return BodyToModelTransform.Inverse() * BodyWorldTransform;
	}

private:
	EAircraftModelForwardAxis ModelForwardAxis = EAircraftModelForwardAxis::PositiveY;
	FTransform BodyToModelTransform = FTransform::Identity;
	bool bValid = true;
};

using UnrealBuildTool;

public class MotionProtoEditorTarget : TargetRules
{
	public MotionProtoEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("MotionProto");
	}
}

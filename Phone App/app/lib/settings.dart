import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// Everything the user sets up once. Stored only on the phone, in the app's
/// private storage (Android backup is switched off for this app), never in
/// the APK or on GitHub. Build-time defaults can be passed with
/// `--dart-define=ORS_API_KEY=... --dart-define=OPENAI_API_KEY=... --dart-define=UPLOAD_URL=...`.
class Settings extends ChangeNotifier {
  static const defaultModel = 'gpt-4.1';

  final SharedPreferences? _prefs;
  Settings._(this._prefs);

  /// Settings that are never saved (tests, previews).
  Settings.memory({
    this.orsKey = '',
    this.openAiKey = '',
    this.openAiModel = defaultModel,
    this.uploadUrl = '',
    this.uploadToken = '',
  }) : _prefs = null;

  String orsKey = const String.fromEnvironment('ORS_API_KEY');
  String openAiKey = const String.fromEnvironment('OPENAI_API_KEY');
  String openAiModel = defaultModel;
  String uploadUrl = const String.fromEnvironment('UPLOAD_URL');
  String uploadToken = '';
  String? deviceId;
  String? deviceName;

  bool get hasOrsKey => orsKey.trim().isNotEmpty;
  bool get hasOpenAiKey => openAiKey.trim().isNotEmpty;
  Uri? get uploadUri {
    final u = Uri.tryParse(uploadUrl.trim());
    return u != null && (u.scheme == 'https' || u.scheme == 'http') && u.host.isNotEmpty ? u : null;
  }

  static Future<Settings> load() async {
    final p = await SharedPreferences.getInstance();
    final s = Settings._(p);
    s.orsKey = p.getString('orsKey') ?? s.orsKey;
    s.openAiKey = p.getString('openAiKey') ?? s.openAiKey;
    s.openAiModel = p.getString('openAiModel') ?? defaultModel;
    s.uploadUrl = p.getString('uploadUrl') ?? s.uploadUrl;
    s.uploadToken = p.getString('uploadToken') ?? '';
    s.deviceId = p.getString('deviceId');
    s.deviceName = p.getString('deviceName');
    return s;
  }

  Future<void> update({
    String? orsKey,
    String? openAiKey,
    String? openAiModel,
    String? uploadUrl,
    String? uploadToken,
  }) async {
    if (orsKey != null) this.orsKey = orsKey.trim();
    if (openAiKey != null) this.openAiKey = openAiKey.trim();
    if (openAiModel != null) this.openAiModel = openAiModel.trim().isEmpty ? defaultModel : openAiModel.trim();
    if (uploadUrl != null) this.uploadUrl = uploadUrl.trim();
    if (uploadToken != null) this.uploadToken = uploadToken.trim();
    final p = _prefs;
    if (p != null) {
      await p.setString('orsKey', this.orsKey);
      await p.setString('openAiKey', this.openAiKey);
      await p.setString('openAiModel', this.openAiModel);
      await p.setString('uploadUrl', this.uploadUrl);
      await p.setString('uploadToken', this.uploadToken);
    }
    notifyListeners();
  }

  Future<void> rememberDevice(String? id, String? name) async {
    deviceId = id;
    deviceName = name;
    final p = _prefs;
    if (p != null) {
      if (id == null) {
        await p.remove('deviceId');
        await p.remove('deviceName');
      } else {
        await p.setString('deviceId', id);
        await p.setString('deviceName', name ?? '');
      }
    }
    notifyListeners();
  }
}

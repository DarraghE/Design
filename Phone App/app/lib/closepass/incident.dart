/// One recorded close pass, as stored on the phone.
enum PlateStatus { waiting, reading, found, uncertain, none, noKey, failed }

enum UploadStatus { notNeeded, notConfigured, waiting, uploading, uploaded, failed }

class PlateResult {
  final bool found;
  final String plate;
  final String country; // IE, UK, OTHER, UNKNOWN
  final double confidence;
  final int bestImage; // 1-based; 0 = none
  final String vehicleType;
  final String vehicleColour;
  final String vehicleMake;
  final String notes;
  final bool plausibleFormat;
  final String model;

  const PlateResult({
    required this.found,
    required this.plate,
    this.country = 'UNKNOWN',
    this.confidence = 0,
    this.bestImage = 0,
    this.vehicleType = '',
    this.vehicleColour = '',
    this.vehicleMake = '',
    this.notes = '',
    this.plausibleFormat = false,
    this.model = '',
  });

  /// Good enough to upload automatically.
  bool get confident => found && plate.isNotEmpty && !plate.contains('?') && confidence >= 0.5;

  String get vehicleSummary =>
      [vehicleColour, vehicleMake, vehicleType].where((s) => s.trim().isNotEmpty && s.toLowerCase() != 'unknown').join(' ');

  Map<String, dynamic> toJson() => {
        'found': found,
        'plate': plate,
        'country': country,
        'confidence': confidence,
        'bestImage': bestImage,
        'vehicleType': vehicleType,
        'vehicleColour': vehicleColour,
        'vehicleMake': vehicleMake,
        'notes': notes,
        'plausibleFormat': plausibleFormat,
        'model': model,
      };

  factory PlateResult.fromJson(Map<String, dynamic> j) => PlateResult(
        found: j['found'] as bool? ?? false,
        plate: j['plate'] as String? ?? '',
        country: j['country'] as String? ?? 'UNKNOWN',
        confidence: (j['confidence'] as num?)?.toDouble() ?? 0,
        bestImage: (j['bestImage'] as num?)?.toInt() ?? 0,
        vehicleType: j['vehicleType'] as String? ?? '',
        vehicleColour: j['vehicleColour'] as String? ?? '',
        vehicleMake: j['vehicleMake'] as String? ?? '',
        notes: j['notes'] as String? ?? '',
        plausibleFormat: j['plausibleFormat'] as bool? ?? false,
        model: j['model'] as String? ?? '',
      );
}

class Incident {
  final String id;
  final DateTime time;
  final double? lat, lng, accuracyM;
  final double? distanceM;
  final double? speedKmh;
  final List<String> photoFiles;
  final List<int> photoOffsetsMs;
  final bool simulated;

  PlateStatus plateStatus;
  PlateResult? plate;
  String? plateError;
  UploadStatus uploadStatus;
  String? uploadError;
  String? uploadedUrl;
  DateTime? uploadedAt;

  Incident({
    required this.id,
    required this.time,
    this.lat,
    this.lng,
    this.accuracyM,
    this.distanceM,
    this.speedKmh,
    this.photoFiles = const [],
    this.photoOffsetsMs = const [],
    this.simulated = false,
    this.plateStatus = PlateStatus.waiting,
    this.plate,
    this.plateError,
    this.uploadStatus = UploadStatus.notNeeded,
    this.uploadError,
    this.uploadedUrl,
    this.uploadedAt,
  });

  bool get hasLocation => lat != null && lng != null;

  /// Index into [photoFiles] of the photo that shows the plate best.
  int get bestPhotoIndex {
    final b = plate?.bestImage ?? 0;
    return b >= 1 && b <= photoFiles.length ? b - 1 : (photoFiles.isEmpty ? -1 : 0);
  }

  Map<String, dynamic> toJson() => {
        'id': id,
        'time': time.toUtc().toIso8601String(),
        'lat': lat,
        'lng': lng,
        'accuracyM': accuracyM,
        'distanceM': distanceM,
        'speedKmh': speedKmh,
        'photoFiles': photoFiles,
        'photoOffsetsMs': photoOffsetsMs,
        'simulated': simulated,
        'plateStatus': plateStatus.name,
        'plate': plate?.toJson(),
        'plateError': plateError,
        'uploadStatus': uploadStatus.name,
        'uploadError': uploadError,
        'uploadedUrl': uploadedUrl,
        'uploadedAt': uploadedAt?.toUtc().toIso8601String(),
      };

  factory Incident.fromJson(Map<String, dynamic> j) {
    T byName<T extends Enum>(List<T> values, Object? name, T fallback) =>
        values.firstWhere((v) => v.name == name, orElse: () => fallback);
    double? d(String k) => (j[k] as num?)?.toDouble();
    return Incident(
      id: j['id'] as String,
      time: DateTime.parse(j['time'] as String).toLocal(),
      lat: d('lat'),
      lng: d('lng'),
      accuracyM: d('accuracyM'),
      distanceM: d('distanceM'),
      speedKmh: d('speedKmh'),
      photoFiles: [for (final f in j['photoFiles'] as List? ?? const []) f as String],
      photoOffsetsMs: [for (final f in j['photoOffsetsMs'] as List? ?? const []) (f as num).toInt()],
      simulated: j['simulated'] as bool? ?? false,
      // A read that was interrupted (app closed) starts again.
      plateStatus: switch (byName(PlateStatus.values, j['plateStatus'], PlateStatus.waiting)) {
        PlateStatus.reading => PlateStatus.waiting,
        final s => s,
      },
      plate: j['plate'] == null ? null : PlateResult.fromJson(j['plate'] as Map<String, dynamic>),
      plateError: j['plateError'] as String?,
      uploadStatus: switch (byName(UploadStatus.values, j['uploadStatus'], UploadStatus.notNeeded)) {
        UploadStatus.uploading => UploadStatus.waiting,
        final s => s,
      },
      uploadError: j['uploadError'] as String?,
      uploadedUrl: j['uploadedUrl'] as String?,
      uploadedAt: j['uploadedAt'] == null ? null : DateTime.parse(j['uploadedAt'] as String).toLocal(),
    );
  }
}

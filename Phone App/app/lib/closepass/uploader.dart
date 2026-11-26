import 'dart:convert';
import 'dart:typed_data';

import 'package:http/http.dart' as http;
import 'package:http_parser/http_parser.dart';

import 'incident.dart';

class UploadResult {
  final String? remoteId;
  final String? url;
  const UploadResult({this.remoteId, this.url});
}

class UploadException implements Exception {
  final String message;
  const UploadException(this.message);
  @override
  String toString() => message;
}

/// Sends a close pass to the website.
///
/// `POST <upload URL>` as `multipart/form-data`, with an optional
/// `Authorization: Bearer <token>` header:
///
/// | Part | Content |
/// |------|---------|
/// | `incident` | JSON (application/json), see [incidentJson] |
/// | `image` | the photo that shows the plate best (image/jpeg) |
/// | `photos` | every photo, in time order (repeated part) |
/// | `lidar` | the LiDAR clip (application/octet-stream, format in close_pass_package.dart) |
///
/// Any 2xx reply counts as success. If the reply is JSON with `id` and/or
/// `url`, the app shows a link to it. The full contract for the website is in
/// Phone App/docs/website_upload.md.
class IncidentUploader {
  final Uri endpoint;
  final String token;
  final http.Client _http;
  final Duration timeout;

  IncidentUploader({required this.endpoint, this.token = '', http.Client? client, this.timeout = const Duration(seconds: 45)})
      : _http = client ?? http.Client();

  static const String appVersion = '1.0.2';

  /// The `incident` part.
  static Map<String, dynamic> incidentJson(Incident i) => {
        'schema': 'bikenav.closepass.v1',
        'id': i.id,
        'recordedAt': i.time.toUtc().toIso8601String(),
        'location': i.hasLocation ? {'lat': i.lat, 'lng': i.lng, 'accuracyM': i.accuracyM} : null,
        'passDistanceM': i.distanceM,
        'passSpeedKmh': i.speedKmh,
        'plate': i.plate?.plate,
        'plateCountry': i.plate?.country,
        'plateConfidence': i.plate?.confidence,
        'plateFormatValid': i.plate?.plausibleFormat,
        'vehicle': i.plate == null
            ? null
            : {'type': i.plate!.vehicleType, 'colour': i.plate!.vehicleColour, 'make': i.plate!.vehicleMake},
        'plateReaderModel': i.plate?.model,
        'photoCount': i.photoFiles.length,
        'bestPhoto': i.bestPhotoIndex + 1,
        'photoOffsetsMs': i.photoOffsetsMs,
        'simulated': i.simulated,
        'app': 'BikeNav Android $appVersion',
      };

  Future<UploadResult> upload(Incident i, {required List<Uint8List> photos, Uint8List? lidar}) async {
    final req = http.MultipartRequest('POST', endpoint);
    if (token.trim().isNotEmpty) req.headers['Authorization'] = 'Bearer ${token.trim()}';
    req.headers['Accept'] = 'application/json';
    req.files.add(http.MultipartFile.fromBytes('incident', utf8.encode(jsonEncode(incidentJson(i))),
        filename: 'incident.json', contentType: MediaType('application', 'json')));
    final best = i.bestPhotoIndex;
    if (best >= 0 && best < photos.length) {
      req.files.add(_image('image', photos[best], i.photoFiles[best]));
    }
    for (var n = 0; n < photos.length; n++) {
      req.files.add(_image('photos', photos[n], i.photoFiles[n]));
    }
    if (lidar != null) {
      req.files.add(http.MultipartFile.fromBytes('lidar', lidar,
          filename: 'lidar.bin', contentType: MediaType('application', 'octet-stream')));
    }
    final http.Response res;
    try {
      res = await http.Response.fromStream(await _http.send(req).timeout(timeout));
    } catch (e) {
      throw UploadException('Could not reach the website (${e.runtimeType}).');
    }
    if (res.statusCode < 200 || res.statusCode >= 300) {
      final body = res.body.length > 120 ? '${res.body.substring(0, 120)}…' : res.body;
      throw UploadException('Website replied ${res.statusCode}${body.trim().isEmpty ? '' : ': $body'}');
    }
    try {
      final j = jsonDecode(res.body);
      if (j is Map) return UploadResult(remoteId: j['id']?.toString(), url: j['url']?.toString());
    } catch (_) {}
    return const UploadResult();
  }

  static http.MultipartFile _image(String field, Uint8List bytes, String filename) {
    final png = filename.endsWith('.png');
    return http.MultipartFile.fromBytes(field, bytes,
        filename: filename, contentType: MediaType('image', png ? 'png' : 'jpeg'));
  }
}
